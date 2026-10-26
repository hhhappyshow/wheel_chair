"""Four sequential 20-inch arrivals: observed pick, online greedy place and return.

No hardware commands. Geometry/FK following and ideal attachment, not dynamics.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import time

MODULE_STARTED=time.perf_counter()

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'perception'))
import cv2
import numpy as np
import newton
from scipy.interpolate import CubicSpline
from scipy import ndimage
from scipy.spatial.transform import Rotation as R,Slerp
from observed_cycle import Runtime
from perception.yolo_sim_perception import render_capture,infer
from perception.envelope_grasp import load_tool,solve_grasp,fit_box
from compare import SCENE


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--pose-json',type=Path,default=HERE/'pose_readback/current_pose_20261024.json')
    p.add_argument('--weights',type=Path,default=HERE/'perception/weights/yolo11s-seg.pt')
    p.add_argument('--free-speed-m-s',type=float,default=.30,help='Desired Cartesian cruise speed, not a hardware limit')
    p.add_argument('--seed',type=int,default=73)
    p.add_argument('--waypoints',type=Path,default=HERE/'pick_waypoints.json')
    p.add_argument('--require-unclipped-view',action='store_true',help='Optional operator-enabled observation gate; off by default')
    p.add_argument('--realtime',action='store_true',help='Pace simulated FK execution at the reference clock for live viewing')
    args=p.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    waypoints=json.loads(args.waypoints.read_text()) if args.waypoints.is_file() else None
    os.environ.setdefault('YOLO_CONFIG_DIR',str(out/'yolo_config'))
    os.environ.setdefault('OMP_NUM_THREADS','2')
    from ultralytics import YOLO
    import torch
    torch.set_num_threads(2)
    detector=YOLO(str(args.weights))
    from suitcase_capture import make_runtime
    catalog=[[.55,.4,.25] for _ in range(4)]
    r=make_runtime(out,catalog)
    pose=json.loads(args.pose_json.read_text());r.q=np.array(pose['q_rad'])
    rng=np.random.default_rng(args.seed)
    for body in r.boxes:
        r.fixed[body]=np.array([100.,100.,100.,0,0,0,1.])
    r.collision.arrived=set();r.collision.current_source=None
    r.history.clear();r.qs.clear();r.stages.clear();r.simtime.clear();r.t=0.
    r.frame(r.q,'MEASURED_EMPTY_START')
    report=dict(completed=False,scope=__doc__,source_catalog_m=catalog,asset_slots=len(catalog),
        initialization_wall_s=time.perf_counter()-MODULE_STARTED,
        initial_measured_pose=str(args.pose_json),grasp_center_offset_xy_m=[0.,0.],
        free_cruise_m_s=args.free_speed_m_s,last_10cm_speed_ratio=.5,
        vacuum='ideal attachment event, no pressure sensor or physical seal validation',
        observation_quality_gate_enabled=args.require_unclipped_view,
        running_simulation=True,wall_clock_paced=args.realtime,
        hardware_motion=False,io_written=False,views=[],events=[],view_adjustments=0)
    shutil.copyfile(SCENE,out/'scene_tf.yaml')
    shutil.copyfile(__file__,out/'source_four20_cycle.py')
    if waypoints:
        shutil.copyfile(args.waypoints,out/'pick_waypoints.json')
        report['waypoints_source']=str(args.waypoints)
    start=time.perf_counter()
    report.update(cycles=[],placements=[],completed_cycles=0,requested_cycles=4,
        luggage_display='clean colored cuboids; textured source actors retained only for RGB-D recognition',
        simulation_evidence='sampled original CAD collision checked FK; ideal attachment and frozen release, not dynamics')
    occupied=[]

    def publish():
        report['wall_s']=time.perf_counter()-start
        report['current_stage']=r.stages[-1] if r.stages else None
        if args.realtime:
            camera_folder=out/'live_camera';camera_folder.mkdir(exist_ok=True)
            r.model.bvh_build_shapes(r.state)
            render_capture(r,camera_folder)
            report['live_camera']=dict(relative_path='live_camera/camera_rgb.png',stamp_s=time.time(),
                source='current native Newton wrist RGB-D sensor; no hardware camera')
        staging=out/'.publishing';staging.mkdir(exist_ok=True)
        previous=r.out;r.out=staging
        try:r.save(report)
        finally:r.out=previous
        os.replace(staging/'trajectory.npz',out/'trajectory.npz')
        os.replace(staging/'result.json',out/'result.json')

    publish()

    def move(poses,stage,speeds=None):
        began=time.perf_counter();q=r.q.copy()
        r.frame(q,stage+'_start')
        publish()
        tcp=r.state.body_q.numpy()[r.tcp]
        last_p,last_r=tcp[:3],tcp[3:]
        knots=[q.copy()];ticks=[0.]
        for k,(position,quat) in enumerate(poses):
            position=np.asarray(position);quat=np.asarray(quat)
            dist=np.linalg.norm(position-last_p)
            angle=(R.from_quat(last_r).inv()*R.from_quat(quat)).magnitude()
            speed=args.free_speed_m_s if speeds is None else speeds[k]
            duration=max(dist/speed,angle/1.,.02)
            samples=max(2,int(np.ceil(duration/.02)))
            rot=Slerp([0.,1.],R.from_quat([last_r,quat]))
            t0=ticks[-1]
            for alpha in np.linspace(0.,1.,samples+1)[1:]:
                ok,q=r.ik._solve_once(last_p*(1-alpha)+position*alpha,rot(alpha).as_quat(),q,check_collision=False)
                if not ok:raise RuntimeError(f'{stage}: IK {r.ik.last_diagnostics}')
                knots.append(q.copy());ticks.append(t0+alpha*duration)
            last_p,last_r=position,quat
        curve=CubicSpline(ticks,knots,bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
        grid=curve(np.linspace(0,ticks[-1],max(2,int(np.ceil(ticks[-1]/.02))+1)))
        for candidate_q in grid:
            if not r.validate(candidate_q):raise RuntimeError(f'{stage}: CAD {r.collision.last_hits}')
        planning_wall_s=time.perf_counter()-began
        execution_start=time.perf_counter()
        for frame,q in enumerate(grid):
            r.frame(q,stage)
            if frame%10==0:publish()
            if args.realtime:time.sleep(max(0.,execution_start+frame*.02-time.perf_counter()))
        report['events'].append(dict(stage=stage,cycle=report['current_cycle'],planning_wall_s=planning_wall_s,
            execution_wall_s=time.perf_counter()-execution_start,reference_s=ticks[-1]))
        publish()

    def taught_pose(name):
        r.ik.model.joint_q.assign(np.array(waypoints[name]['q_rad']))
        newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
        return r.ik.model.body_q.numpy()[r.ik.contact_body].copy()

    def observe(view,locked=None,previous=None):
        observation_started=time.perf_counter()
        folder=out/f'view{view}';folder.mkdir()
        r.model.bvh_build_shapes(r.state)
        rgb,cloud=render_capture(r,folder)
        record,mask,fitted=infer(detector,rgb,cloud,device='cpu',locked_center_world=locked)
        if locked is not None and previous is not None:
            # A lost semantic label does not invalidate an already locked
            # metric target. Track visible elevated depth components, not IDs.
            ground=np.array(previous['ground_plane'])
            finite=np.isfinite(cloud).all(axis=2)
            elevated=finite & (cloud@ground[:3]+ground[3]>.07) & (cloud[:,:,2]<.8)
            labels,count=ndimage.label(elevated)
            components=[]
            for k in range(1,count+1):
                pixels=labels==k
                if pixels.sum()<3:continue
                points=cloud[pixels]
                distance=float(np.linalg.norm(np.median(points,axis=0)-locked))
                components.append((distance,pixels))
            if components:
                distance,selected=min(components,key=lambda row:row[0])
                fitted=fit_box(cloud[selected][::2],ground)
                # The planar interior of a rounded lid is smaller than its
                # visible boundary. Use observed boundary extents, not padding
                # or simulator dimensions, for the metric box footprint.
                old_tf=np.array(fitted['T_world_box']);top=np.array(fitted['top_center_world_m'])
                points=cloud[selected];xy=(points-top)@old_tf[:3,:2]
                corners=cv2.boxPoints(cv2.minAreaRect(xy.astype('float32'))).astype(float)
                world_corners=top+corners@old_tf[:3,:2].T
                edges=np.roll(world_corners,-1,axis=0)-world_corners
                lengths=np.linalg.norm(edges,axis=1);axis=edges[lengths.argmax()]/lengths.max()
                if axis[0]<0:axis=-axis
                normal=old_tf[:3,2];top=world_corners.mean(axis=0);height=fitted['dimensions_m'][2]
                tf_box=np.eye(4);tf_box[:3,:3]=np.column_stack((axis,np.cross(normal,axis),normal))
                tf_box[:3,3]=top-normal*height/2
                vertices=np.r_[world_corners,world_corners-normal*height]
                fitted.update(dimensions_m=[float(lengths.max()),float(lengths.min()),height],
                    T_world_box=tf_box.tolist(),top_center_world_m=top.tolist(),top_corners_world_m=world_corners.tolist(),
                    aabb_min_world_m=vertices.min(0).tolist(),aabb_max_world_m=vertices.max(0).tolist(),
                    yaw_world_deg=float(np.degrees(np.arctan2(axis[1],axis[0]))),
                    footprint_source='full visible target-depth boundary projected onto fitted top plane; no padding or GT dimensions')
                fitted['selection_source']='YOLO-initialized world lock, current elevated depth component; no fresh semantic target detection'
                clipped=bool(selected[:3].any() or selected[-3:].any() or selected[:,:3].any() or selected[:,-3:].any())
                record.update(status='LOCKED_TARGET_DEPTH_TRACKED',fitted=fitted,
                    target_selection='previous YOLO world lock, nearest elevated observed depth component',
                    semantic_re_detection=False,track_distance_m=distance,gt_mask_used=False,
                    selection_source=fitted['selection_source'],detection_confidence=None,
                    completeness=dict(image_border_clipped=clipped),detection_bbox_xyxy=None)
                cv2.imwrite(str(folder/'locked_depth_selection.png'),selected.astype('uint8')*255)
        cv2.imwrite(str(folder/'yolo_overlay.png'),infer.last_overlay)
        (folder/'result.json').write_text(json.dumps(record,indent=2))
        record.update(cycle=report.get('current_cycle'),trace_index=len(r.history)-1,view_index=view,
            observation_wall_s=time.perf_counter()-observation_started)
        report['views'].append(record)
        publish()
        return record,fitted,cloud

    try:
        for cycle in range(4):
            cycle_started=time.perf_counter()
            report['current_cycle']=cycle+1
            body=r.boxes[cycle]
            r.fixed[body]=np.r_[[.30,1.15,.125],R.from_euler('z',rng.uniform(-.30,.30)).as_quat()]
            r.collision.arrived.add(cycle);r.collision.current_source=cycle
            r.frame(r.q,'CURRENT_ARRIVAL_REVEALED')
            publish()
            tcp=r.state.body_q.numpy()[r.tcp].copy()
            if waypoints:
                observation=np.array(waypoints['observation']['tcp_world_pose_xyzw'])
                move([(observation[:3],observation[3:])],'ABSOLUTE_OBSERVATION')
            else:
                move([(tcp[:3]+[0,0,.10],tcp[3:])],'RAISE_OBSERVATION_10CM')
            # Ground scene generation belongs to simulation; no pose/size oracle is
            # passed to detection or the metric fit.
            r.frame(r.q,'OBSERVE_GROUND')
            record,fitted,cloud=observe(len(report['views']))
            if fitted is None:raise RuntimeError(record['status'])
            locked=np.array(fitted['top_center_world_m'])
            bbox=np.array(record['detection_bbox_xyxy']);center=(bbox[:2]+bbox[2:])/2
            if np.linalg.norm(center-[320,180])>1.:
                camera=r.state.body_q.numpy()[r.camera]
                axis=R.from_quat(camera[3:]).apply([1.,0.,0.])
                plane=np.array(fitted['top_plane'])
                optical=camera[:3]+axis*(-(camera[:3]@plane[:3]+plane[3])/(axis@plane[:3]))
                delta=locked-optical
                delta[2]=.10 if record['completeness']['image_border_clipped'] else 0.
                tcp=r.state.body_q.numpy()[r.tcp].copy()
                move([(tcp[:3]+delta,tcp[3:])],'ONE_TARGET_VIEW_ADJUSTMENT')
                report['view_adjustments']=1
                record,fitted,cloud=observe(len(report['views']),locked,fitted)
                if fitted is None:raise RuntimeError('Locked target not recovered after single adjustment')
            report['selected_top_image_clipped']=record['completeness']['image_border_clipped']
            if args.require_unclipped_view and report['selected_top_image_clipped']:
                raise RuntimeError('Selected top remains clipped after the one allowed adjustment')
            tool=HERE.parent/'s20_greedy_async_20261023/v2_s20_model_h100/s20_current.urdf'
            parts,plate,fixture,_=load_tool(tool)
            cfg=r.cfg;tf=next(t for t in cfg['static_transforms'] if t['child']=='container_link')
            grasp=solve_grasp(fitted,parts,plate,fixture,centered=True,
                preferred_fixture_direction_world=R.from_euler('xyz',tf['rotation_rpy']).apply([1.,0.,0.]))
            current_rot=R.from_quat(r.state.body_q.numpy()[r.tcp,3:])
            choices=[c for c in grasp['candidates'] if c['contact_plate_fits']]
            for c in choices:
                c['rotation_from_observation_rad']=(current_rot.inv()*R.from_matrix(np.array(c['T_world_tcp'])[:3,:3])).magnitude()
            # Equivalent plate-edge headings should not introduce an unnecessary
            # half-turn; footprint containment remains separate from preference.
            choice=min(choices,key=lambda c:(not c['full_envelope_fits'],round(c['max_overhang_m'],5),
                c['rotation_from_observation_rad']+.5*(1-c['preferred_fixture_direction_score']))) if choices else None
            if choice is None:raise RuntimeError('Contact plate does not fit fitted top surface')
            report['grasp']=choice
            report['grasp_heading_comparison']=[{k:c[k] for k in ('angle_deg','rotation_from_observation_rad',
                'preferred_fixture_direction_score','max_overhang_m')} for c in choices]
            mat=np.array(choice['T_world_tcp']);top=mat[:3,3];quat=R.from_matrix(mat[:3,:3]).as_quat()
            # Use the fitted observed box center to associate the simulator actor
            # only when emulating contact; actor truth never supplies the target.
            estimated=np.array(fitted['T_world_box'])[:3,3]
            body=min(r.boxes,key=lambda b:np.linalg.norm(r.fixed[b][:3]-estimated))
            index=r.boxes.index(body);r.collision.current_source=index
            normal=mat[:3,2]*-1.
            move([(top+normal*.10,quat),(top,quat)],'CENTER_ALIGN_AND_CONTACT',
                 speeds=[args.free_speed_m_s,args.free_speed_m_s*.5])
            tcp=r.state.body_q.numpy()[r.tcp];contact_tcp=tcp.copy();box=r.fixed[body]
            r.w.grip_offset=R.from_quat(tcp[3:]).inv().apply(box[:3]-tcp[:3])
            r.w.grip_rot=R.from_quat(tcp[3:]).inv()*R.from_quat(box[3:])
            r.w.held=body;r.frame(r.q,'IDEAL_VACUUM_ON')
            report['ideal_attachment_actor']=r.model.body_label[body]
            move([(top+[0,0,.20],quat)],'LIFT_20CM')
            if waypoints:
                lift=taught_pose('post_pick_lift')
                move([(lift[:3],lift[3:])],'OPERATOR_TAUGHT_LIFT_POINT')
                report['terminal_point']='operator_taught_post_pick_lift; container entry not yet supplied'
                if waypoints.get('container_entry'):
                    entry=taught_pose('container_entry')
                    move([(entry[:3],entry[3:])],'OPERATOR_TAUGHT_CONTAINER_ENTRY')
                    report['terminal_point']='operator_taught_container_entry; placement not yet executed'
            from straight_place.cycle_place import place_and_return
            if not (out/'source_cycle_place.py').exists():
                shutil.copyfile(HERE/'straight_place/cycle_place.py',out/'source_cycle_place.py')
            placed=place_and_return(r,fitted,contact_tcp,occupied,move,taught_pose,publish,report,realtime=args.realtime)
            occupied.append(placed)
            report['terminal_point']='observation_after_release_and_empty_return'
            report['completed_cycles']=cycle+1
            report['cycles'].append(dict(cycle=cycle+1,wall_s=time.perf_counter()-cycle_started,
                completed=True,release_model='frozen actual pose',returned_to_observation=True))
            publish()
        report['completed']=True
    except KeyboardInterrupt:
        report['error']='Operator/developer interrupted this simulation attempt; partial trace retained'
    except Exception as exc:
        report['error']=str(exc)
    finally:
        report['running_simulation']=False
        report['wall_s']=time.perf_counter()-start
        publish()
        print(json.dumps({k:report.get(k) for k in ('completed','error','view_adjustments','wall_s')},indent=2))

if __name__=='__main__':main()

