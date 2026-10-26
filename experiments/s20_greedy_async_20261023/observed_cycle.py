"""Fresh wrist RGB-D -> fitted box -> async current-item plan -> full FK cycle.

No object-ID/depth ground-truth mask or incoming dimensions enter perception.
Known CAD/calibration and pickup workspace are priors. Ideal joint following,
ideal attachment and frozen releases: NOT a dynamics or hardware certificate.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import os
import shutil
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R,Slerp
from scipy import ndimage
import yaml
from compare import ROOT,UPSTREAM,SCENE,p,top_surface_model
from newton_compare import TopWorkspace,Collision
from online_packing_newton import make_model
from scene_frames import container_pose,rotation
import newton
from newton.sensors import SensorTiledCamera
import warp as wp


def placement_worker(request_path,output,regions,compact_prior=False):
    import contextlib
    from newton_compare import main as plan_main
    start=time.perf_counter()
    command=[str(Path(__file__).with_name("newton_compare.py")),"--request",request_path,
             "--output",output,"--count","1","--endpoint-first","--release-contact",
             "--straight","--placement-only","--ik-tolerance-m",".01"]
    if regions:command.extend(["--regions","--batch-endpoints","--no-endpoint-hints"])
    if compact_prior:command.extend(["--compact-prior","--batch-endpoints","--no-endpoint-hints"])
    with (Path(output).parent/"placement_worker.log").open("w") as log:
        previous=sys.argv;sys.argv=command
        try:
            with contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):plan_main()
            code=0
        except Exception:
            import traceback
            traceback.print_exc(file=log);code=1
        finally:sys.argv=previous
    return dict(code=code,wall_s=time.perf_counter()-start,scope="persistent Python worker, models still initialized per-query; no nested process or unused empty route")


class Runtime:
    def __init__(self,out,realtime,catalog=None):
        self.out=out;self.realtime=realtime;wp.set_device("cuda:0")
        self.cfg=yaml.safe_load(SCENE.read_text());self.hull=p.Hull.from_config(self.cfg)
        self.ik=TopWorkspace(UPSTREAM/"policy/vendor/roboticarm",SCENE,device="cuda:0");self.ik.position_tolerance_m=.01
        catalog=catalog or [[.55,.4,.25]]*3
        sample=dict(container=self.hull.as_dict(),steps=[dict(incoming=dict(id=f"B{i+1}",dims=d)) for i,d in enumerate(catalog)])
        self.model,ids,_,_=make_model(sample,UPSTREAM/"policy/vendor/roboticarm",SCENE);self.state=self.model.state()
        self.boxes=list(ids.values());self.tcp=self.model.body_label.index("S20/suction_contact_frame")
        self.camera=self.model.body_label.index("S20/camera_link")
        self.w=SimpleNamespace(model=self.model,state=self.state,fk=self.model.state(),boxes=self.boxes,
                               q=self.model.joint_q.numpy(),qidx=np.arange(6),arm_count=min(self.boxes),tcp=self.tcp,
                               held=None,grip_offset=np.array([0,0,.125]),grip_rot=R.from_euler("x",np.pi))
        self.collision=Collision(self.w);flags=self.model.shape_flags.numpy()
        for shape in list(self.collision.objs):
            b=self.collision.bodies[shape]
            if b>=0 and self.model.body_label[b].endswith("/payload_box"):
                del self.collision.objs[shape];self.collision.arm.discard(shape);flags[shape]=0
        self.model.shape_flags.assign(flags)
        bundle=UPSTREAM/"policy/demo_data/site_four_colors_right160_observe120/trajectory.npz"
        saved=np.load(bundle);idx=np.flatnonzero((saved["box_id"]=="B2")&(saved["stage"]=="APPROACH"))[0]
        self.q=saved["joint_q"][idx].copy();self.initial_q=self.q.copy()
        self.fixed={self.boxes[0]:np.array([0.,1.6,catalog[0][2]/2,0,0,0,1.])}
        for b in self.boxes[1:]:self.fixed[b]=np.array([100.,100.,100.,0,0,0,1.])
        self.history=[];self.qs=[];self.stages=[];self.simtime=[];self.t=0.;self.events=[];self.capture_count=0
        self.frame(self.q,"OBSERVE")
        self.collision.current_source=0
        self.sensor=SensorTiledCamera(self.model);self.sensor.utils.create_default_light(enable_shadows=False)
        self.rays=self.sensor.utils.compute_pinhole_camera_rays(640,360,2*np.arctan(180/322.8994))
        self.depth=self.sensor.utils.create_depth_image_output(640,360)
        self.color=self.sensor.utils.create_color_image_output(640,360)
        self.model.bvh_build_shapes(self.state)

    def frame(self,q,stage):
        self.q=np.array(q).copy();self.w.q[:6]=q;self.state.joint_q.assign(self.w.q)
        newton.eval_fk(self.model,self.state.joint_q,self.state.joint_qd,self.state)
        poses=self.state.body_q.numpy()
        for b,pose in self.fixed.items():poses[b]=pose
        if self.w.held is not None:
            tcp=poses[self.tcp];r=R.from_quat(tcp[3:]);poses[self.w.held]=np.r_[tcp[:3]+r.apply(self.w.grip_offset),(r*self.w.grip_rot).as_quat()]
        self.state.body_q.assign(poses)
        self.history.append(poses.copy());self.qs.append(self.q.copy());self.stages.append(stage);self.simtime.append(self.t);self.t+=.02

    def capture_model(self):
        start=time.perf_counter();camera=self.state.body_q.numpy()[self.camera]
        # Newton GL rays use -Z; camera_link convention is +X forward.
        gl=R.from_matrix([[0,0,-1],[-1,0,0],[0,1,0]])
        cr=R.from_quat(camera[3:])*gl;origin=camera[:3]+R.from_quat(camera[3:]).apply([.025,0,0])
        pose=wp.array([[wp.transformf(wp.vec3(*origin),wp.quat(*cr.as_quat()))]],dtype=wp.transformf)
        self.sensor.update(self.state,pose,self.rays,depth_image=self.depth,color_image=self.color)
        depth=self.depth.numpy()[0,0];rgba=self.sensor.utils.to_rgba_from_color(self.color).numpy()
        rgba=rgba.reshape(360,640,4);cv2.imwrite(str(self.out/"camera_rgb.png"),rgba[:,:,:3][:,:,::-1])
        cv2.imwrite(str(self.out/f"camera_rgb_{self.capture_count}.png"),rgba[:,:,:3][:,:,::-1])
        rays=self.rays.numpy()[0,:,:,1];points=cr.apply((rays*depth[:,:,None]).reshape(-1,3))+origin
        sensor_s=time.perf_counter()-start
        # A fixed pickup WORKSPACE, not a hidden object's bbox or shape ID.
        mask=(depth.reshape(-1)>0)&(abs(points[:,0])<.75)&(points[:,1]>1.)&(points[:,1]<2.4)&(points[:,2]>-.03)&(points[:,2]<.8)
        cloud=points[mask];start=time.perf_counter()
        floor=cloud[cloud[:,2]<np.quantile(cloud[:,2],.35)]
        center=floor.mean(axis=0);_,_,vt=np.linalg.svd(floor-center,full_matrices=False);n=vt[-1]
        if n[2]<0:n=-n
        plane=np.r_[n,-center@n]
        height=points@plane[:3]+plane[3]
        object_mask=(mask&(height>.07)).reshape(360,640)
        components,count=ndimage.label(object_mask)
        if not count:raise RuntimeError("No elevated connected depth object in pickup workspace")
        sizes=np.bincount(components.ravel());sizes[0]=0;selected=components==sizes.argmax()
        sample_pixels=np.zeros((360,640),dtype=bool);sample_pixels[::2,::2]=True
        model=top_surface_model(points[(selected&sample_pixels).ravel()],plane)
        model_s=time.perf_counter()-start
        clipped=bool(selected[0].any() or selected[-1].any() or selected[:,0].any() or selected[:,-1].any())
        camera_forward=cr.apply([0,0,-1]);plane_z=model["top_center_world_m"][2]
        image_center_world=origin+camera_forward*(plane_z-origin[2])/camera_forward[2]
        np.savez_compressed(self.out/"camera_observation.npz",depth_range_m=depth,points_world=points.reshape(360,640,3),selected=selected,ground_plane=plane)
        cv2.imwrite(str(self.out/"selected_mask.png"),selected.astype(np.uint8)*255)
        model.update(sensor_s=sensor_s,model_s=model_s,source="native Newton wrist RGBD; geometric connected component, NO GT mask; known pickup workspace",
                     ground_plane=plane.tolist(),camera_gl_origin=origin.tolist(),camera_gl_xyzw=cr.as_quat().tolist(),
                     image_border_clipped=clipped,image_center_on_top_plane_world=image_center_world.tolist())
        (self.out/"observed_box.json").write_text(json.dumps(model,indent=2))
        (self.out/f"observed_box_view{self.capture_count}.json").write_text(json.dumps(model,indent=2));self.capture_count+=1
        return model

    def validate(self,q,allow_final=False):
        filters=self.collision.filters.copy()
        try:
            while not self.collision.validate(q):
                import fcl
                pair=self.collision.last_hits[0];ids=[self.model.shape_label.index(s) for s in pair]
                result=fcl.CollisionResult();fcl.collide(self.collision.objs[ids[0]],self.collision.objs[ids[1]],fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
                cargo=all(s.startswith("B") and s.endswith("_collision_box") for s in pair)
                if not(allow_final and cargo and max(c.penetration_depth for c in result.contacts)<=.001):return False
                self.collision.filters.add(tuple(sorted(ids)))
            return True
        finally:self.collision.filters=filters

    def execute(self,grid,stage):
        start=time.perf_counter()
        for i,q in enumerate(grid):
            if not self.validate(q,allow_final=stage=="PLACE" and i==len(grid)-1):raise RuntimeError(f"{stage}: collision {self.collision.last_hits}")
            self.frame(q,stage)
            if self.realtime:time.sleep(max(0.,start+(i+1)*.02-time.perf_counter()))
        return time.perf_counter()-start

    def move(self,poses,stage,duration):
        start=time.perf_counter();q=self.q.copy();self.ik.model.joint_q.assign(q)
        newton.eval_fk(self.ik.model,self.ik.model.joint_q,self.ik.model.joint_qd,self.ik.model)
        tcp=self.ik.model.body_q.numpy()[self.ik.contact_body];last=(tcp[:3],tcp[3:]);knots=[q]
        for position,quat in poses:
            position=np.array(position);quat=np.array(quat)
            angle=(R.from_quat(last[1]).inv()*R.from_quat(quat)).magnitude()
            n=max(1,int(np.ceil(np.linalg.norm(position-last[0])/.03)),int(np.ceil(angle/.08)))
            slerp=Slerp([0,1],R.from_quat([last[1],quat]))
            for alpha in np.linspace(0,1,n+1)[1:]:
                target=last[0]*(1-alpha)+position*alpha;rotation=slerp(alpha).as_quat()
                ok,q=self.ik._solve_once(target,rotation,q,check_collision=False)
                if not ok:raise RuntimeError(f"{stage} IK {self.ik.last_diagnostics}")
                knots.append(q.copy())
            last=(position,quat)
        distances=np.r_[0,np.cumsum(np.maximum(np.max(abs(np.diff(knots,axis=0)),axis=1),1e-6))]
        curve=CubicSpline(distances/distances[-1]*duration,knots,bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
        grid=curve(np.linspace(0,duration,int(duration/.02)+1))
        planning=time.perf_counter()-start;execution=self.execute(grid,stage)
        self.events.append(dict(stage=stage,planning_s=planning,execution_wall_s=execution,reference_s=duration))

    def save(self,result):
        np.savez_compressed(self.out/"trajectory.npz",body_q=np.array(self.history),joint_q=np.array(self.qs),stage=np.array(self.stages),body_labels=np.array(self.model.body_label),times=np.array(self.simtime))
        (self.out/"result.json").write_text(json.dumps(result,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--mode",choices=("async","serial"),default="async");parser.add_argument("--regions",action="store_true")
    parser.add_argument("--compact-prior",action="store_true")
    parser.add_argument("--sensor-only",action="store_true");parser.add_argument("--unpaced",action="store_true")
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    for name in ("observed_cycle.py","newton_compare.py","compare.py","region_search.py","batch_endpoints.py","compact_prior.py"):
        shutil.copyfile(Path(__file__).with_name(name),args.output/("source_"+name))
    shutil.copyfile(SCENE,args.output/"scene_tf.yaml")
    start=time.perf_counter();runtime=Runtime(args.output,not args.unpaced);initialization=time.perf_counter()-start
    result=dict(scope=__doc__,mode=args.mode,initialization_s=initialization,completed=False,events=runtime.events,hardware_motion=False,wall_clock_paced=not args.unpaced)
    pool=None
    try:
        views=[]
        for _ in range(3):
            observed=runtime.capture_model();views.append(observed)
            if not observed["image_border_clipped"]:break
            if args.sensor_only:break
            tcp=runtime.state.body_q.numpy()[runtime.tcp]
            delta=np.array(observed["top_center_world_m"])-np.array(observed["image_center_on_top_plane_world"]);delta[2]=.25
            runtime.move([(tcp[:3]+delta,tcp[3:])],"ADJUST_VIEW",2.)
        result["perception"]=observed;result["views"]=views
        if observed["image_border_clipped"] and not args.sensor_only:raise RuntimeError("Selected surface still clipped after view adjustment")
        if args.sensor_only:result["sensor_only"]=True;return
        request=dict(dimensions_m=observed["dimensions_m"],T_world_box=observed["T_world_box"],scene=str(SCENE),scene_version=0,future_arrivals_used=0)
        request_path=args.output/"request.json";request_path.write_text(json.dumps(request))
        # Start a persistent worker BEFORE approach/contact/lift, never hold
        # the robot simulation until a placement search finishes unnecessarily.
        import multiprocessing as mp
        pool=ProcessPoolExecutor(max_workers=1,mp_context=mp.get_context("spawn"))
        dispatch=time.perf_counter();future=pool.submit(placement_worker,str(request_path),str(args.output/"placement"),args.regions,args.compact_prior)
        if args.mode=="serial":
            before=time.perf_counter();result["worker"]=future.result();result["serial_pre_pick_wait_s"]=time.perf_counter()-before
        box_tf=np.array(observed["T_world_box"]);top=np.array(observed["top_center_world_m"])
        target_rotation=R.from_matrix(box_tf[:3,:3])*R.from_euler("x",np.pi)
        quat=target_rotation.as_quat()
        # Rectangles have a 180deg symmetry: select the equivalent grasp
        # closest to the actual starting tool orientation, not a wrist spin.
        current=R.from_quat(runtime.state.body_q.numpy()[runtime.tcp,3:])
        options=[target_rotation,target_rotation*R.from_euler("z",np.pi)]
        quat=min(options,key=lambda r:(current.inv()*r).magnitude()).as_quat()
        runtime.move([(top+[0,0,.15],quat)],"APPROACH",3.)
        runtime.move([(top,quat)],"PICK_CONTACT",1.)
        tcp=runtime.state.body_q.numpy()[runtime.tcp];actual=runtime.state.body_q.numpy()[runtime.boxes[0]]
        runtime.w.grip_offset=R.from_quat(tcp[3:]).inv().apply(actual[:3]-tcp[:3]);runtime.w.grip_rot=R.from_quat(tcp[3:]).inv()*R.from_quat(actual[3:])
        runtime.w.held=runtime.boxes[0];runtime.frame(runtime.q,"IDEAL_VACUUM_ON")
        result["grasp_gap_to_true_top_m"]=float(np.linalg.norm(tcp[:3]-(actual[:3]+[0,0,.125])))
        runtime.move([(top+[0,0,.18],quat)],"LIFT",1.5)
        # Independent of candidate endpoint: lift above pedestal, then head
        # around its outside toward the fixed entrance-side staging region.
        high=np.array([top[0],1.20,1.25]);runtime.move([(high,quat)],"RAISE_FOR_TRANSFER",3.)
        dims=tuple(observed['dimensions_m'])
        staging_position,staging_yaw=runtime.ik.local_contact_world((-dims[0]-.10,.33,.4),dims,0.,runtime.hull)
        staging_rotation=runtime.ik.tool_orientation(staging_yaw)
        # Align across the OUTSIDE rail before approaching the doorway; a
        # diagonal from the side cuts across the actual door-frame post.
        runtime.move([([0.,1.15,1.25],quat),([-.30,1.10,1.25],staging_rotation),
                      ([-.30,staging_position[1],staging_position[2]],staging_rotation),
                      (staging_position,staging_rotation)],"CARRY_TO_ENTRANCE",6.)
        ready=time.perf_counter();result["plan_ready_on_entrance_arrival"]=future.done();result["pick_transfer_wall_since_dispatch_s"]=ready-dispatch
        worker=future.result();result["worker"]=worker;result["wait_at_entrance_s"]=time.perf_counter()-ready
        if worker["code"]:raise RuntimeError("Placement worker failed; inspect placement_worker.log")
        plan=json.loads((args.output/"placement/result.json").read_text());result["placement_report"]=plan
        if plan.get("loaded_references",0)!=1 and plan["placed"]!=1:raise RuntimeError("No placement route found")
        data=np.load(args.output/"placement/trajectory.npz");indices=np.flatnonzero(data["stage"]=="B1_insert");entry_q=data["joint_q"][indices[0]]
        runtime.ik.model.joint_q.assign(entry_q);newton.eval_fk(runtime.ik.model,runtime.ik.model.joint_q,runtime.ik.model.joint_qd,runtime.ik.model)
        entry=runtime.ik.model.body_q.numpy()[runtime.ik.contact_body];entry_rotation=entry[3:]
        result["staging_wait_s"]=result["wait_at_entrance_s"]
        # Continue on the actual reached branch, rather than teleporting to
        # the worker's joint seed. The task-space route is what is consumed.
        cycle=plan["cycles"][0];dims=cycle["box"]["dims"];yaw=runtime.ik.box_yaw(tuple(observed["dimensions_m"]),tuple(dims))
        targets=[(entry[:3],entry_rotation)]
        for point in cycle["route"][1:]:
            pos,wyaw=runtime.ik.local_contact_world(point,dims,yaw,runtime.hull);targets.append((pos,runtime.ik.tool_orientation(wyaw)))
        runtime.move(targets,"PLACE",6.)
        actual=runtime.state.body_q.numpy()[runtime.boxes[0]].copy();runtime.fixed[runtime.boxes[0]]=actual
        runtime.w.held=None;runtime.collision.current_source=None;runtime.collision.arrived={0};runtime.frame(runtime.q,"RELEASE_FROZEN")
        runtime.move([(entry[:3],entry_rotation)],"EMPTY_WITHDRAW",4.)
        runtime.move([([-.30,entry[1],entry[2]],entry_rotation),([-.30,1.10,1.25],entry_rotation),([0.,1.15,1.25],quat),(high,quat)],"RETURN_TO_OBSERVATION",6.)
        result.update(completed=True,cycle_reference_s=runtime.t)
    except Exception as exc:
        result["failure"]=dict(reason=str(exc),stage=runtime.stages[-1],hits=runtime.collision.last_hits)
        print(json.dumps(result["failure"]),flush=True)
    finally:
        if pool is not None:pool.shutdown(wait=True)
        result.update(wall_s=time.perf_counter()-start,sim_reference_s=runtime.t,frames=len(runtime.qs))
        runtime.save(result);print(json.dumps({k:v for k,v in result.items() if k not in ("perception","placement_report","views")}),flush=True)


if __name__=="__main__":main()
