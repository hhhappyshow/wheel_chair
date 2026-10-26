"""Consume a freshly computed background reference on the actual simulation state."""
from copy import deepcopy
from dataclasses import asdict
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import newton
from compare import Box


def revalidate_plan(r, plan, scene_version):
    began=time.perf_counter()
    if not plan.get('ok'):
        return None,dict(accepted=False,reason='worker_failed',worker_error=plan.get('error'))
    if plan['scene_version']!=scene_version:
        return None,dict(accepted=False,reason='scene_version_changed')
    qs=np.asarray(plan['joint_q'],float);stages=list(plan['stages'])
    held=r.w.held;source=r.collision.current_source
    original=r.state.body_q.numpy().copy()
    offset=np.array(r.w.grip_offset);rotation=r.w.grip_rot
    difference=float(np.max(abs(qs[0]-r.q)))
    count=max(2,int(np.ceil(difference/.02))+1)
    bridge=np.linspace(r.q,qs[0],count)[1:-1]
    qs=np.concatenate([bridge,qs]);stages=['ASYNC_ENTRY_BRANCH_BRIDGE']*len(bridge)+stages
    detail=dict(accepted=False,scene_version=scene_version,entry_joint_difference_rad=difference,
        samples=len(qs),attachment_source='actual post-contact attachment; background prediction not trusted as actual',
        intended_release_contact='Original suction/source contact only during separation; strict CAD afterward')
    previous=None
    try:
        for index,(q,stage) in enumerate(zip(qs,stages)):
            if previous and previous.endswith('_EMPTY_SEPARATE') and stage!=previous:
                r.collision.current_source=None
                if not r.validate(qs[index-1]):
                    detail.update(reason='strict_detachment_clearance_failed',sample=index-1,pairs=r.collision.last_hits)
                    return None,detail
            if 'RELEASE_ACTUAL_FROZEN' in stage:
                r.w.fk.joint_q.assign(q)
                newton.eval_fk(r.model,r.w.fk.joint_q,r.w.fk.joint_qd,r.w.fk)
                tcp=r.w.fk.body_q.numpy()[r.tcp];rot=R.from_quat(tcp[3:])
                poses=original.copy()
                poses[held]=np.r_[tcp[:3]+rot.apply(offset),(rot*rotation).as_quat()]
                r.state.body_q.assign(poses);r.w.held=None;r.collision.current_source=None
                previous=stage
                continue
            if r.w.held is None:
                r.collision.current_source=source if stage.endswith('_EMPTY_SEPARATE') else None
            if not r.validate(q):
                detail.update(reason='actual_attachment_CAD_rejected',stage=stage,sample=index,pairs=r.collision.last_hits)
                return None,detail
            previous=stage
        detail.update(accepted=True,reason='all_actual_state_samples_passed')
        return (qs,stages),detail
    finally:
        detail['wall_s']=time.perf_counter()-began
        r.state.body_q.assign(original);r.w.held=held;r.collision.current_source=source


def commit_plan(r, plan, reference, report, publish, realtime=False):
    qs,stages=reference;held=r.w.held;source=r.collision.current_source
    item=deepcopy(plan['placement'])
    item.update(completed=False,phase='executing_revalidated_background_plan',events=[],
        background_reference=True,actual_arrival_revalidation=True,
        simulated_actual_grip_offset_m=np.asarray(r.w.grip_offset).tolist(),
        simulated_actual_grip_rotation_xyzw=r.w.grip_rot.as_quat().tolist())
    report['placements'].append(item)
    released=None;began=time.perf_counter();cursor=0
    while cursor<len(qs):
        stage=stages[cursor];end=cursor+1
        while end<len(qs) and stages[end]==stage:end+=1
        if 'RELEASE_ACTUAL_FROZEN' in stage:
            released=r.state.body_q.numpy()[held].copy()
            r.fixed[held]=released;r.w.held=None;r.collision.current_source=None
        elif r.w.held is None:
            r.collision.current_source=source if stage.endswith('_EMPTY_SEPARATE') else None
        execution=time.perf_counter()
        for i,q in enumerate(qs[cursor:end]):
            r.frame(q,stage)
            if realtime and i%10==0:publish()
            if realtime:time.sleep(max(0.,execution+(i+1)*.02-time.perf_counter()))
        event=dict(stage=stage,frames=end-cursor,reference_s=(end-cursor)*.02,
            execution_wall_s=time.perf_counter()-execution,planning_source='fresh concurrent worker reference')
        item['events'].append(event);report['events'].append(event)
        cursor=end;publish()
    if released is None:raise RuntimeError('Background reference has no release event')
    tf=next(v for v in r.cfg['static_transforms'] if v['child']=='container_link')
    cr=R.from_euler('xyz',tf['rotation_rpy']);translation=np.array(tf['translation'])
    h=r.hull;origin=np.array([h.door_x,h.inner_y0,h.floor_z])
    center=cr.inv().apply(released[:3]-translation)-origin
    extent=abs((cr.inv()*R.from_quat(released[3:])).as_matrix())@np.array(item.get('planning_dimensions_m',[.55,.4,.25]))
    occupied=Box(r.model.body_label[held],tuple(extent),tuple(center-extent/2))
    item.update(completed=True,phase='returned_observation',actual_frozen_pose_world=released.tolist(),
        occupied=asdict(occupied),empty_return_completed=True,actual_execution_wall_s=time.perf_counter()-began,
        occupancy_source='Actual frozen released proxy OBB, not the worker predicted release')
    publish()
    return occupied
