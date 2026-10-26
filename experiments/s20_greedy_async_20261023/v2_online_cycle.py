"""Current native RGB-D box -> online greedy -> V2 lift/entrance/place/retreat.

Prescribed FK execution, ideal vacuum and frozen actual releases, not dynamics.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import signal
import time

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

from compare import SCENE, Box
from compact_prior import CompactPrior
from observed_cycle import Runtime
from v2_backend import Backend
import newton


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--count",type=int,default=8)
    parser.add_argument("--seed",type=int,default=73)
    parser.add_argument("--ideal-transfer",action="store_true",help="Ideal held box at the entrance; omit perception and ground-pick/transfer")
    parser.add_argument("--cartesian-repair",action="store_true",help="Simulation-only official Newton IK straight-line repair; explicitly not pure V2 output")
    parser.add_argument("--resume-from",type=Path,help="Resume the exact completed FK prefix, not another layout or physical dynamics state")
    args=parser.parse_args()
    def interrupted(signum,frame):raise InterruptedError(f"External signal {signum}")
    signal.signal(signal.SIGTERM,interrupted)
    out=args.output;out.mkdir(parents=True,exist_ok=False)
    catalog=np.round(np.array([.55,.4,.25])*np.random.default_rng(args.seed).uniform(.9,1.1,(args.count,3)),3).tolist()
    started=time.perf_counter();r=Runtime(out,False,catalog=catalog)
    if args.ideal_transfer:
        entry=np.load(Path(__file__).parent/"compact_contact_motion3_final/trajectory.npz")
        r.q=entry["joint_q"][np.flatnonzero(entry["stage"]=="B1_insert")[0]].copy()
        r.initial_q=r.q.copy();r.history.clear();r.qs.clear();r.stages.clear();r.simtime.clear();r.t=0.
        r.frame(r.q,"IDEAL_ENTRY_INITIALIZATION")
    backend=Backend(r.model,r.state,Path(__file__).parent/"v2_s20_model_h100",out,r.q)
    cfg=r.cfg;ct=cfg["static_transforms"][0];cr=R.from_euler("xyz",ct["rotation_rpy"]);translation=np.array(ct["translation"])
    h=r.hull;origin=np.array([h.door_x,h.inner_y0,h.floor_z])
    prior=CompactPrior(cfg,h,max_box_top_m=1.,depth_step_m=.1);prior.horizontal_first=True;prior.row_first=True
    prior.contact_first=True
    placed=[];fixed={};cycles=[];events=[];current=None
    previous_wall=0.
    if args.resume_from:
        old=json.loads((args.resume_from/"result.json").read_text())
        if old["source_catalog_m"]!=catalog:raise ValueError("Arrival dimensions differ from checkpoint")
        saved=np.load(args.resume_from/"trajectory.npz")
        if list(saved["body_labels"])!=r.model.body_label:raise ValueError("Checkpoint model order differs")
        cycles=[c for c in old["cycles"] if c["status"]=="complete"]
        events=list(old["events"]);previous_wall=old["wall_s"]
        for c in cycles:
            b=Box(**c["actual_occupancy"]);placed.append(b)
            fixed[r.boxes[c["arrival"]-1]]=np.array(c["actual_frozen_pose_world"])
        end=np.flatnonzero(saved["stage"]==f"B{len(cycles)}_return_observe")[-1]+1
        r.history=list(saved["body_q"][:end]);r.qs=list(saved["joint_q"][:end]);r.stages=list(saved["stage"][:end])
        r.simtime=list(saved["times"][:end]);r.t=float(r.simtime[-1])+.02
        r.q=r.qs[-1].copy();r.w.q[:6]=r.q;r.state.joint_q.assign(r.w.q);r.state.body_q.assign(r.history[-1])
    result=dict(scope=__doc__,scene=str(SCENE),source_catalog_m=catalog,seed=args.seed,
        max_box_top_m=1.,endpoint_sphere_m=1.7,completed=False,placed=0,cycles=cycles,events=events,
        input="Native synthetic RGB-D, selected metric surface; no GT dimensions/mask fed to placement",
        timing="Space worker wall time; V2 queries separately; CAD validation and observation separately",
        attachment_proxy="Box-contained spheres with 1mm numerical inset: inner approximation with gaps; original CAD unchanged and sampled checks retained",
        validation="20ms full CAD + V2 sphere/self checks, not continuous GSIP proof or physical settling",
        initialization_s=time.perf_counter()-started)
    if args.ideal_transfer:
        result.update(input="Ideal current-item dimensions, held-entry initialization; no camera or ground-pick/transfer execution",
                      ideal_transfer=True,scope="cuRoboV2 entry-place-release-empty-return with ideal grasp/frozen releases and prescribed FK; not dynamics")
    result["cartesian_repair_enabled"]=args.cartesian_repair
    if args.resume_from:result["resume"]=dict(source=str(args.resume_from),completed_prefix=len(cycles),previous_wall_s=previous_wall,scope="Exact saved FK states/occupancy; no dynamics checkpoint claim")
    for name in ("v2_online_cycle.py","v2_backend.py","compact_prior.py","observed_cycle.py","compare.py"):
        shutil.copy2(Path(__file__).with_name(name),out/("source_"+name))
    shutil.copy2(SCENE,out/"scene_tf.yaml")
    shutil.copy2(Path(__file__).parent/"v2_s20_model_h100/robot.yml",out/"robot.yml")
    root_out=out;pool=ThreadPoolExecutor(max_workers=1);query_cache={};unresolved_cache={}

    def save():
        result["wall_s"]=previous_wall+time.perf_counter()-started
        result["placed"]=sum(c["status"]=="complete" for c in cycles)
        r.out=root_out;r.save(result)

    def obstacles(include_source=False,exclude=None):
        rows=[(b.id,catalog[int(b.id[1:])-1],fixed[r.boxes[int(b.id[1:])-1]]) for b in placed if b.id!=exclude]
        if include_source and current is not None:
            rows.append((f"B{current+1}",catalog[current],r.fixed[r.boxes[current]]))
        return rows

    def valid_cad(q,allow_contact=False):
        if not allow_contact:return r.validate(q)
        import fcl
        original=r.collision.filters.copy()
        try:
            while not r.collision.validate(q):
                pair=r.collision.last_hits[0]
                cargo=[name for name in pair if name.startswith("B") and name.endswith("_collision_box")]
                if not cargo or not all(name in cargo or name=="container_collision_full" for name in pair):return False
                indices=[r.model.shape_label.index(name) for name in pair]
                contact=fcl.CollisionResult()
                fcl.collide(r.collision.objs[indices[0]],r.collision.objs[indices[1]],fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),contact)
                depth=max((c.penetration_depth for c in contact.contacts),default=float("inf"))
                events.append(dict(stage="cargo_contact_check",pairs=pair,depth_m=float(depth),admitted=bool(depth<=.001)))
                if depth>.001:return False
                r.collision.filters.add(tuple(sorted(indices)))
            return True
        finally:r.collision.filters=original

    def query(q,position,quat,phase,goal_q=None):
        allow_contact="_candidate" in phase
        allow_repair=args.cartesian_repair and (allow_contact or "_empty_exit" in phase or "_separate" in phase)
        key=(current,len(placed),r.w.held,tuple(np.asarray(q,dtype=np.float32)),
             None if position is None else tuple(np.asarray(position,dtype=np.float32)),
             None if quat is None else tuple(np.asarray(quat,dtype=np.float32)),
             None if goal_q is None else tuple(np.asarray(goal_q,dtype=np.float32)))
        if key in query_cache:
            events.append(dict(stage=phase,wall_s=0.,success=True,cache="same prefix, attachment, exact start and target"))
            return query_cache[key].copy()
        if key in unresolved_cache:
            raise RuntimeError(f"Same-state finite-budget unresolved: {unresolved_cache[key]}")
        def cartesian_repair(samples):
            r.ik.model.joint_q.assign(q)
            newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
            initial=r.ik.model.body_q.numpy()[r.ik.contact_body]
            rotations=Slerp([0.,1.],R.from_quat([initial[3:],quat]))
            repaired=[np.array(q).copy()];previous=np.array(q).copy()
            for t in np.linspace(0.,1.,samples)[1:]:
                s=t*t*t*(10.+t*(-15.+6.*t))
                ok,previous=r.ik._solve_once(initial[:3]*(1-s)+np.array(position)*s,rotations(s).as_quat(),previous,check_collision=False)
                if not ok or not valid_cad(previous,allow_contact):return None
                repaired.append(previous.copy())
            return np.array(repaired)
        seed=None;seed_traj=None;ik_start=time.perf_counter()
        if goal_q is None:
            for initial in (q,r.ik.q_default):
                ok,candidate=r.ik._solve_once(position,quat,initial,check_collision=False)
                if ok and valid_cad(candidate,allow_contact):seed=candidate;break
            if seed is None:
                for candidate in backend.endpoint_seeds(q,position,quat):
                    if valid_cad(candidate,allow_contact):seed=candidate;break
                if seed is None:
                    elapsed=time.perf_counter()-ik_start
                    events.append(dict(stage=phase,wall_s=elapsed,success=False,reason="endpoint IK unresolved before trajectory optimization"))
                    unresolved_cache[key]=phase
                    raise RuntimeError(f"Endpoint IK unresolved at {phase}")
            if seed is not None:
                r.ik.model.joint_q.assign(q)
                newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
                initial=r.ik.model.body_q.numpy()[r.ik.contact_body]
                initial_rot=R.from_quat(initial[3:]);goal_rot=R.from_quat(quat)
                angle=(initial_rot.inv()*goal_rot).magnitude()
                if r.w.held is not None:
                    axis=None
                    if allow_contact and not phase.endswith("leg0") and angle<.05:
                        axis=int(np.argmax(np.abs(goal_rot.inv().apply(np.array(position)-initial[:3]))))
                    backend.hold_tool_down(True,freeze_yaw=angle<.05,free_translation_axis=axis)
                n=max(2,int(np.ceil(np.linalg.norm(np.array(position)-initial[:3])/.06)),int(np.ceil(angle/.12)))
                slerp=Slerp([0,1],R.from_quat([initial[3:],quat]));knots=[np.array(q).copy()];qi=np.array(q).copy()
                for alpha in np.linspace(0,1,n+1)[1:]:
                    ok,qi=r.ik._solve_once(initial[:3]*(1-alpha)+np.array(position)*alpha,slerp(alpha).as_quat(),qi,check_collision=False)
                    if not ok:break
                    knots.append(qi.copy())
                else:
                    knots=np.array(knots);grid=np.linspace(0,len(knots)-1,backend.planner.trajopt_solver.action_horizon)
                    seed_traj=np.stack([np.interp(grid,np.arange(len(knots)),knots[:,j]) for j in range(6)],axis=1)
                    seed=seed_traj[-1]
        ik_s=time.perf_counter()-ik_start
        path,timing=backend.plan(q,position,quat,goal_q=goal_q,seed_q=seed,seed_traj=seed_traj)
        timing["seed_ik_s"]=ik_s
        row=dict(stage=phase,**timing)
        events.append(row)
        if path is None and allow_repair:
            repair_start=time.perf_counter();path=cartesian_repair(201)
            row.update(cartesian_repair_s=time.perf_counter()-repair_start,engine="V2 unresolved -> Newton Cartesian IK" if path is not None else "V2 and Cartesian unresolved")
        if path is None:
            unresolved_cache[key]=phase
            raise RuntimeError(f"V2 unresolved at {phase}")
        t=time.perf_counter()
        for qi in path:
            if not valid_cad(qi,allow_contact):
                np.savez_compressed(root_out/f"rejected_{len(events):06d}_{phase}.npz",joint_q=path,
                    actual_scene_body_q=r.state.body_q.numpy(),body_labels=np.array(r.model.body_label),
                    rejected_q=qi,target_position=position,target_quaternion=quat)
                if allow_repair:
                    repair_start=time.perf_counter();repaired=cartesian_repair(max(len(path),201))
                    row["cartesian_repair_s"]=time.perf_counter()-repair_start
                    if repaired is not None:
                        path=repaired;row["engine"]="V2 CAD rejected -> Newton Cartesian IK";break
                raise RuntimeError(f"CAD rejected {phase}: {r.collision.last_hits}")
        row["cad_validation_s"]=time.perf_counter()-t
        query_cache[key]=path.copy()
        return path

    def move(position,quat,phase,goal_q=None):
        path=query(r.q,position,quat,phase,goal_q)
        # Already checked against this exact fixed prefix and attachment.
        for qi in path:r.frame(qi,phase)
        print(json.dumps(dict(stage=phase,frames=len(path))),flush=True)
        save()
        return path

    def space_worker(dims,prefix):
        began=time.perf_counter();rows,stats=prior.candidates(dims,prefix)
        routes=[]
        from compare import p
        for b in rows:
            transit_height=max([.4]+[v.pos[2]+v.dims[2]+.05 for v in prefix])
            seen=set()
            for minimum in (transit_height,.4):
                route=p.route_to(b,prefix,h,min_entry_z=minimum)
                key=None if route is None else tuple(route)
                if route is not None and key not in seen:routes.append((b,route));seen.add(key)
        return routes,dict(wall_s=time.perf_counter()-began,**stats)

    try:
        for index in range(len(cycles),args.count):
            current=index;label=f"B{index+1}";event_start=len(events)
            cycle=dict(arrival=index+1,status="running");cycles.append(cycle)
            r.fixed={body:pose.copy() for body,pose in fixed.items()}
            for body in r.boxes:
                if body not in r.fixed:r.fixed[body]=np.array([100.,100.,100.,0,0,0,1.])
            r.fixed[r.boxes[index]]=np.r_[[0.,1.6,catalog[index][2]/2],[0,0,0,1.]]
            r.w.held=None;r.collision.current_source=index;r.collision.arrived=set(range(len(placed)))
            r.frame(r.q,f"{label}_arrival");backend.detach();backend.hold_tool_down(False)
            backend.update_world(obstacles(include_source=True))
            if args.ideal_transfer:
                dims=tuple(catalog[index]);cycle.update(observation_and_model_s=0.,modeled_dimensions_m=dims)
                r.w.grip_offset=np.array([0.,0.,dims[2]/2]);r.w.grip_rot=R.from_euler("x",np.pi)
                backend.update_world(obstacles());grasp=r.state.body_q.numpy()[r.tcp,3:]
            else:
                capture=out/"perception"/label;capture.mkdir(parents=True);r.out=capture;r.capture_count=0
                obs_started=time.perf_counter()
                for view in range(3):
                    r.out=capture
                    r.model.bvh_build_shapes(r.state);observed=r.capture_model()
                    if not observed["image_border_clipped"]:break
                    tcp=r.state.body_q.numpy()[r.tcp]
                    delta=np.array(observed["top_center_world_m"])-observed["image_center_on_top_plane_world"];delta[2]=.2
                    move(tcp[:3]+delta,tcp[3:],f"{label}_adjust_view")
                if observed["image_border_clipped"]:raise RuntimeError(f"{label} surface clipped")
                cycle["observation_and_model_s"]=time.perf_counter()-obs_started
                dims=tuple(np.round(observed["dimensions_m"],3));cycle["modeled_dimensions_m"]=dims
                top=np.array(observed["top_center_world_m"])
                rotation=R.from_matrix(np.array(observed["T_world_box"])[:3,:3])*R.from_euler("x",np.pi)
                now=R.from_quat(r.state.body_q.numpy()[r.tcp,3:])
                grasp=min([rotation,rotation*R.from_euler("z",np.pi)],key=lambda rot:(now.inv()*rot).magnitude()).as_quat()
                move(top+[0,0,.15],grasp,f"{label}_approach")
                backend.update_world(obstacles());move(top,grasp,f"{label}_contact")
                tcp=r.state.body_q.numpy()[r.tcp];box=r.state.body_q.numpy()[r.boxes[index]]
                r.w.grip_offset=R.from_quat(tcp[3:]).inv().apply(box[:3]-tcp[:3])
                r.w.grip_rot=R.from_quat(tcp[3:]).inv()*R.from_quat(box[3:])
            r.w.held=r.boxes[index];r.frame(r.q,f"{label}_ideal_vacuum_on")
            backend.attach(dims,r.q,r.w.grip_offset,r.w.grip_rot);backend.hold_tool_down(True)
            future=pool.submit(space_worker,dims,list(placed));dispatched=time.perf_counter()
            if not args.ideal_transfer:
                move(top+[0,0,.20],grasp,f"{label}_lift")
                move([top[0],1.20,1.25],grasp,f"{label}_raise_above_base")
                move([0.,1.15,1.25],grasp,f"{label}_outside_transfer")
            ready=time.perf_counter();candidates,stats=future.result()
            cycle.update(space_planning_s=stats["wall_s"],space_stats=stats,
                         async_wait_s=time.perf_counter()-ready,
                         pickup_transfer_overlap_window_s=ready-dispatched)
            save()
            selected=None;failures=[]
            sensor_ids=[j for j,name in enumerate(r.model.body_label) if name.endswith(("/camera_link","/mid360_mount_frame"))]
            bodies=r.state.body_q.numpy();contact=bodies[r.tcp]
            sensor_offset=R.from_quat(contact[3:]).inv().apply(bodies[sensor_ids,:3].mean(axis=0)-contact[:3])
            for rank,(b,route) in enumerate(candidates):
                yaw=-np.pi/2 if b.dims[0]<b.dims[1] else 0.
                toward=cr.apply([0,h.width/2-b.pos[1]-b.dims[1]/2,0])
                def inward(offset):
                    rot=cr*R.from_euler("z",yaw+offset)*r.w.grip_rot.inv()
                    return float(rot.apply(sensor_offset)@toward)
                for heading in sorted([0.,np.pi],key=lambda offset:-inward(offset)):
                    target_rot=cr*R.from_euler("z",yaw+heading)
                    def target(point):
                        center=translation+cr.apply(np.array(point)+np.array(b.dims)/2+origin)
                        tool_rot=target_rot*r.w.grip_rot.inv()
                        return center-tool_rot.apply(r.w.grip_offset),tool_rot.as_quat()
                    reachable=True;endpoint_started=time.perf_counter()
                    for point in route[1:]:
                        point=np.array(point).copy()
                        if np.allclose(point,route[-1]):point[2]+=.005
                        position,quat=target(point);ok=False
                        for initial in (r.q,r.ik.q_default):
                            solved,qend=r.ik._solve_once(position,quat,initial,check_collision=False)
                            if solved and valid_cad(qend,True):ok=True;break
                        if not ok:ok=any(valid_cad(qend,True) for qend in backend.endpoint_seeds(r.q,position,quat))
                        if not ok:reachable=False;break
                    events.append(dict(stage=f"{label}_candidate{rank}_endpoint",wall_s=time.perf_counter()-endpoint_started,success=reachable))
                    if not reachable:
                        failures.append(dict(rank=rank,heading_deg=float(np.degrees(heading)),reason="mandatory endpoint IK unresolved"))
                        continue
                    entry_position,entry_quat=target(route[0])
                    for transit in (None,-.65,-.45,-.30):
                        qi=r.q.copy();paths=[]
                        try:
                            if transit is not None:
                                for rail,rail_position in enumerate(([transit,entry_position[1],1.25],[transit,entry_position[1],entry_position[2]])):
                                    path=query(qi,rail_position,entry_quat,f"{label}_candidate{rank}_rail{rail}")
                                    paths.append(path);qi=path[-1]
                            for leg,point in enumerate(route):
                                point=np.array(point).copy()
                                if leg==len(route)-1:point[2]+=.005
                                position,quat=target(point)
                                path=query(qi,position,quat,f"{label}_candidate{rank}_leg{leg}")
                                paths.append(path);qi=path[-1]
                            selected=b,paths,rank,heading
                            break
                        except RuntimeError as exc:
                            failures.append(dict(rank=rank,heading_deg=float(np.degrees(heading)),transit_x=transit,reason=str(exc)))
                            print(json.dumps(failures[-1]),flush=True)
                    if selected:break
                if selected:break
            cycle["candidate_failures"]=failures
            if selected is None:raise RuntimeError(f"{label}: all current candidate paths unresolved")
            b,paths,rank,heading=selected
            for leg,path in enumerate(paths):
                for qi in path:r.frame(qi,f"{label}_insert_leg{leg}")
            actual=r.state.body_q.numpy()[r.boxes[index]].copy()
            fixed[r.boxes[index]]=actual;r.fixed[r.boxes[index]]=actual;r.w.held=None
            backend.detach();backend.hold_tool_down(False)
            center=cr.inv().apply(actual[:3]-translation)-origin
            extent=np.abs((cr.inv()*R.from_quat(actual[3:])).as_matrix())@np.array(dims)
            occupied=Box(label,tuple(np.round(extent,3)),tuple(np.round(center-extent/2,3)));placed.append(occupied)
            r.collision.arrived=set(range(len(placed)));r.frame(r.q,f"{label}_release_actual_frozen")
            # Separate the suction contact before adding the just-released
            # box to the V2 empty-return world; CAD still includes the box.
            backend.update_world(obstacles(exclude=label))
            tcp=r.state.body_q.numpy()[r.tcp]
            separated=False;separation_failures=[]
            for option,delta in enumerate(([0,0,.15],cr.apply([-.15,0,0]),cr.apply([-.10,0,.05]),[0,0,.08])):
                try:
                    move(tcp[:3]+delta,tcp[3:],f"{label}_separate{option}");separated=True;break
                except RuntimeError as exc:separation_failures.append(str(exc))
            cycle["separation_failures"]=separation_failures
            if not separated:raise RuntimeError(f"{label}: no empty separation path; {separation_failures}")
            r.collision.current_source=None;backend.update_world(obstacles())
            exited=False;exit_failures=[]
            for exit_height in dict.fromkeys((.65,min(h.door_z1-.10,max(v.pos[2]+v.dims[2] for v in placed)+.15),.95)):
                exit_position=translation+cr.apply(origin+[-.35,(h.door_y0+h.door_y1)/2,exit_height])
                try:
                    move(exit_position,tcp[3:],f"{label}_empty_exit");exited=True;break
                except RuntimeError as exc:exit_failures.append(str(exc))
            cycle["empty_exit_failures"]=exit_failures
            if not exited:raise RuntimeError(f"{label}: no empty doorway exit; {exit_failures}")
            if not args.ideal_transfer:move([-.30,1.10,1.25],grasp,f"{label}_outside_return")
            lower=r.ik.model.joint_limit_lower.numpy()[:6];upper=r.ik.model.joint_limit_upper.numpy()[:6]
            goal=r.initial_q.copy()
            for j in range(6):
                equivalent=[goal[j]+k*2*np.pi for k in range(-2,3) if lower[j]<=goal[j]+k*2*np.pi<=upper[j]]
                if equivalent:goal[j]=min(equivalent,key=lambda value:abs(value-r.q[j]))
            move(None,None,f"{label}_return_observe",goal_q=goal)
            cycle.update(status="complete",nominal=asdict(b),actual_occupancy=asdict(occupied),chosen_candidate=rank,
                equivalent_heading_deg=float(np.degrees(heading)),actual_frozen_pose_world=actual.tolist(),
                trajectory_planning_s=sum(e.get("wall_s",0.)+e.get("seed_ik_s",0.)+e.get("cartesian_repair_s",0.) for e in events if e["stage"].startswith(label+"_")),
                cad_validation_s=sum(e.get("cad_validation_s",0.) for e in events[event_start:]))
            save();print(json.dumps(cycle),flush=True)
        result["completed"]=True
    except Exception as exc:
        result["error"]=str(exc)
        if cycles and cycles[-1]["status"]=="running":cycles[-1]["status"]="failed"
        print(json.dumps(dict(error=str(exc))),flush=True)
    finally:
        pool.shutdown(wait=True);save();backend.planner.destroy()


if __name__=="__main__":main()
