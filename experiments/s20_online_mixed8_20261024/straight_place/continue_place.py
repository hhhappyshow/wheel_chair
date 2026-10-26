"""Single observed-box placement from an exact saved held-entry state.

Independent simulation continuation, not an uninterrupted replay of pickup.
"""
import argparse
import csv
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import sys
import time

sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
PARENT=HERE.parent
ROOT=PARENT.parents[1]
sys.path[:0]=[str(PARENT),str(PARENT/'perception')]
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
for key,sub in [('WARP_CACHE_PATH','warp_cache'),('XDG_CACHE_HOME','cache'),('MPLCONFIGDIR','mpl_config')]:
    os.environ[key]=str(HERE/sub)
import numpy as np
from scipy.spatial.transform import Rotation as R,Slerp
import warp as wp
wp.config.kernel_cache_dir=str(HERE/'warp_cache')
from suitcase_capture import make_runtime
from compact_prior import CompactPrior
from compare import p
import newton


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=PARENT/'pick_to_entry_attempt1')
    parser.add_argument('--output',type=Path,default=HERE/'attempt1')
    parser.add_argument('--max-candidates',type=int,default=0,help='0 checks whole regenerated pool until first accepted path')
    args=parser.parse_args()
    if not args.output.resolve().is_relative_to(HERE) or args.output.resolve()==HERE:
        parser.error('Output must be inside straight_place')
    args.output.mkdir(parents=True,exist_ok=False);out=args.output
    started=time.perf_counter();source=args.source.resolve()
    old=json.loads((source/'result.json').read_text());saved=np.load(source/'trajectory.npz')
    if not old['completed'] or not str(saved['stage'][-1]).startswith('OPERATOR_TAUGHT_CONTAINER_ENTRY'):
        raise ValueError('Source must be a completed saved held-entry chain')
    fit=old['views'][-1]['fitted']
    shutil.copy2(source/'scene_tf.yaml',out/'scene_tf.yaml')
    dims=tuple(float(v) for v in fit['dimensions_m'])
    # Actor dimensions are confined to scene construction and CAD validation.
    r=make_runtime(out,old['source_catalog_m'])
    assert r.model.body_label==list(saved['body_labels'])
    r.history.clear();r.qs.clear();r.stages.clear();r.simtime.clear();r.t=0.
    r.q=saved['joint_q'][-1].copy();poses=saved['body_q'][-1].copy()
    r.w.q[:6]=r.q;r.state.joint_q.assign(r.w.q);r.state.body_q.assign(poses)
    held=r.model.body_label.index(old['ideal_attachment_actor']);tcp=poses[r.tcp]
    actual_rot=R.from_quat(tcp[3:]).inv()*R.from_quat(poses[held,3:])
    actual_offset=R.from_quat(tcp[3:]).inv().apply(poses[held,:3]-tcp[:3])
    r.w.grip_offset=actual_offset;r.w.grip_rot=actual_rot;r.w.held=held
    r.fixed={body:poses[body].copy() for body in r.boxes}
    r.collision.arrived=set(range(len(r.boxes)));r.collision.current_source=r.boxes.index(held)
    contact_index=int(np.flatnonzero(saved['stage']=='IDEAL_VACUUM_ON')[-1])
    contact=saved['body_q'][contact_index,r.tcp]
    fit_matrix=np.array(fit['T_world_box'])
    estimated_rot=R.from_quat(contact[3:]).inv()*R.from_matrix(fit_matrix[:3,:3])
    estimated_offset=R.from_quat(contact[3:]).inv().apply(fit_matrix[:3,3]-contact[:3])
    r.frame(r.q,'RESTORED_EXACT_HELD_ENTRY')
    restored_error=float(np.max(abs(r.history[0]-poses)))
    # Keep the exact saved initial body frame rather than a recomputed near copy.
    r.history[0]=poses.copy();r.state.body_q.assign(poses)
    tf=next(v for v in r.cfg['static_transforms'] if v['child']=='container_link')
    cr=R.from_euler('xyz',tf['rotation_rpy']);translation=np.array(tf['translation'])
    h=r.hull;origin=np.array([h.door_x,h.inner_y0,h.floor_z])
    report=dict(completed=False,hardware_motion=False,io_written=False,
        scope='One-box geometric FK simulation continuation; ideal held actor, no contact dynamics',
        source=str(source),source_last_stage=str(saved['stage'][-1]),source_last_q=saved['joint_q'][-1].tolist(),
        source_frame_count=len(saved['stage']),pickup_prefix_reexecuted=False,uninterrupted_run=False,
        body_labels_match=True,initial_restoration_fk_max_abs_error=restored_error,
        planning_dimensions_m=dims,planning_dimensions_source='last saved observed view fitted dimensions; no GT dimensions supplied to candidate/target functions',
        planning_footprint_source=fit.get('footprint_source',fit.get('source')),
        planning_observed_T_world_box=fit_matrix.tolist(),planning_estimated_grip_offset_m=estimated_offset.tolist(),
        planning_estimated_grip_rotation_xyzw=estimated_rot.as_quat().tolist(),
        simulation_actual_grip_offset_m=actual_offset.tolist(),simulation_actual_grip_rotation_xyzw=actual_rot.as_quat().tolist(),
        actor_source_catalog_m=old['source_catalog_m'],actor_geometry_used_for='scene/emulation and original CAD collision only',
        source_catalog_m=old['source_catalog_m'],asset_slots=len(old['source_catalog_m']),
        free_cruise_m_s=.3,reference_tick_s=.02,events=[],failures=[],selected=None,
        release='No dynamics; stop held at checked final approach. Release only if full checked placement succeeds, saved as frozen actual pose.',
        limits='Existing native Newton joint limits/tolerances and original CAD checker; no new joint or quality limits')
    timings=[]

    def cad(q):
        if r.validate(q):return None
        import fcl
        hits=[]
        for pair in r.collision.last_hits:
            indices=[r.model.shape_label.index(v) for v in pair];result=fcl.CollisionResult()
            fcl.collide(r.collision.objs[indices[0]],r.collision.objs[indices[1]],fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
            hits.append(dict(pair=pair,bodies=[r.model.body_label[r.collision.bodies[i]] if r.collision.bodies[i]>=0 else 'world' for i in indices],
                max_depth_m=max((float(c.penetration_depth) for c in result.contacts),default=None)))
        return hits

    def segment(qstart,position,quat,stage):
        began=time.perf_counter();r.ik.model.joint_q.assign(qstart)
        newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
        initial=r.ik.model.body_q.numpy()[r.ik.contact_body].copy()
        distance=float(np.linalg.norm(np.array(position)-initial[:3]));angle=float((R.from_quat(initial[3:]).inv()*R.from_quat(quat)).magnitude())
        duration=max(distance/.3,angle,.02);count=max(2,int(np.ceil(duration/.02)))
        slerp=Slerp([0,1],R.from_quat([initial[3:],quat]));q=np.array(qstart).copy();path=[]
        for index,alpha in enumerate(np.linspace(0,1,count+1)[1:]):
            target=initial[:3]*(1-alpha)+np.array(position)*alpha
            ok,q=r.ik._solve_once(target,slerp(alpha).as_quat(),q,check_collision=False)
            if not ok:
                error=dict(reason='native_ik_unsolved',sample=index,target_world_m=target.tolist(),diagnostics=dict(r.ik.last_diagnostics),q_rad=q.tolist())
                break
            error=cad(q)
            if error:
                error=dict(reason='original_cad_rejected',sample=index,target_world_m=target.tolist(),pairs=error,q_rad=q.tolist())
                break
            path.append(q.copy())
        else:error=None
        row=dict(stage=stage,wall_s=time.perf_counter()-began,reference_s=duration,samples=len(path),success=error is None)
        timings.append(row)
        if error:return None,error
        return np.array(path),None

    def commit(path,stage):
        for q in path:r.frame(q,stage)
        report['events'].append(dict(stage=stage,frames=len(path),reference_s=len(path)*.02))

    try:
        initial_error=cad(r.q)
        if initial_error:raise RuntimeError(f'Restored entry already CAD rejected: {initial_error}')
        entry_tcp=r.state.body_q.numpy()[r.tcp].copy()
        lower,error=segment(r.q,entry_tcp[:3]+[0,0,-.05],entry_tcp[3:],'OPTIONAL_ENTRY_LOWER_5CM')
        if error:report['failures'].append(dict(stage='OPTIONAL_ENTRY_LOWER_5CM',**error))
        else:commit(lower,'OPTIONAL_ENTRY_LOWER_5CM')
        base_q=r.q.copy()
        prior=CompactPrior(r.cfg,h,max_box_top_m=1.,depth_step_m=.1)
        prior.horizontal_first=True;prior.row_first=True;prior.contact_first=True
        began=time.perf_counter();candidates,stats=prior.candidates(dims,[])
        timings.append(dict(stage='GREEDY_SPACE',wall_s=time.perf_counter()-began,reference_s=0,samples=len(candidates),success=True))
        report['candidate_stats']=stats;report['candidate_count']=len(candidates)
        selected=None
        for rank,b in enumerate(candidates):
            if args.max_candidates and rank>=args.max_candidates:break
            yaw=-np.pi/2 if np.allclose(b.dims,(dims[1],dims[0],dims[2])) else 0.
            routes=[]
            for min_z in (.4,0.):
                route=p.route_to(b,[],h,min_entry_z=min_z)
                if route is not None and route not in routes:routes.append(route)
            current_r=R.from_quat(r.state.body_q.numpy()[r.tcp,3:])
            headings=sorted((0.,np.pi),key=lambda a:(current_r.inv()*cr*R.from_euler('z',yaw+a)*estimated_rot.inv()).magnitude())
            for heading in headings:
                box_r=cr*R.from_euler('z',yaw+heading);tool_r=box_r*estimated_rot.inv()
                def target(point):
                    center=translation+cr.apply(np.array(point)+np.array(b.dims)/2+origin)
                    return center-tool_r.apply(estimated_offset),tool_r.as_quat()
                for route_index,route in enumerate(routes):
                    q=base_q.copy();segments=[];error=None
                    # Heading is aligned before translating into the doorway.
                    r.ik.model.joint_q.assign(q);newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
                    outside_tcp=r.ik.model.body_q.numpy()[r.ik.contact_body,:3].copy()
                    goals=[('ALIGN_HEADING_OUTSIDE',outside_tcp,tool_r.as_quat())]
                    for leg,point in enumerate(route):
                        point=np.array(point).copy()
                        if leg==len(route)-1:point[2]+=.005
                        position,quat=target(point)
                        goals.append((f'ROUTE_LEG_{leg}',position,quat))
                    for phase,position,quat in goals:
                        path,error=segment(q,position,quat,f'candidate{rank}_heading{int(np.degrees(heading))}_route{route_index}_{phase}')
                        if error:break
                        segments.append((phase,path));q=path[-1]
                    if error:
                        report['failures'].append(dict(rank=rank,candidate=asdict(b),heading_deg=float(np.degrees(heading)),route=route,
                            stage=phase,**error))
                    else:
                        selected=(b,rank,heading,route,segments);break
                if selected:break
            if selected:break
            if rank%10==0:print(json.dumps(dict(candidate=rank,failed_attempts=len(report['failures']))),flush=True)
        if selected is None:raise RuntimeError('Finite current greedy candidate/heading/straight-route search exhausted; see retained failures')
        b,rank,heading,route,segments=selected
        for phase,path in segments:commit(path,phase)
        report['selected']=dict(rank=rank,box=asdict(b),heading_deg=float(np.degrees(heading)),route=route,
                                final_release_offset_container_z_m=.005)
        final=r.state.body_q.numpy()[held].copy()
        # Frozen release preserves actual actor pose; does not imply settling.
        r.fixed[held]=final;r.w.held=None;r.frame(r.q,'RELEASE_ACTUAL_FROZEN')
        report.update(completed=True,actual_frozen_pose_world=final.tolist(),
                      terminal_point='one observed B1 placed with frozen release; no retreat or next item executed')
    except Exception as exc:
        report['error']=str(exc)
    finally:
        report.update(wall_s=time.perf_counter()-started,frames=len(r.history),timings=timings)
        r.save(report)
        with (out/'times.csv').open('w',newline='') as file:
            writer=csv.DictWriter(file,fieldnames=['stage','wall_s','reference_s','samples','success']);writer.writeheader();writer.writerows(timings)
        print(json.dumps({k:report.get(k) for k in ('completed','error','candidate_count','selected','wall_s','frames')},indent=2),flush=True)


if __name__=='__main__':main()
