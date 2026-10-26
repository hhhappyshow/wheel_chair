"""Bind observed pick/place references to explicit, unperformed hardware events.

This portable adapter never connects to a controller or commits predicted boxes.
It uses the existing ServoJ resampler without smoothing across vacuum events.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pick',type=Path,required=True)
    p.add_argument('--place',type=Path,help='Omit to prepare only contact/vacuum/lift/entry')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    pick_report=json.loads((a.pick/'result.json').read_text())
    place=json.loads((a.place/'result.json').read_text()) if a.place else None
    request=json.loads((a.place/'request.json').read_text()) if a.place else None
    observation=json.loads(Path(pick_report['observation']).read_text())
    if not pick_report['success'] or place is not None and not place['success']:
        raise ValueError('Both input planners must have succeeded')
    if place is not None and Path(pick_report['observation']).resolve()!=Path(place['observation_source']).resolve():
        raise ValueError('Pick and place must use the same current observation')
    if place is not None and Path(pick_report['site_urdf']).resolve()!=Path(place['site_urdf']).resolve():
        raise ValueError('Pick and place must use the same robot model')
    pick=np.load(a.pick/'reference.npz',allow_pickle=False)
    q=pick['joint_q'];t=pick['times_s'];stages=pick['stage']
    qd=pick['joint_qd'] if 'joint_qd' in pick else None
    if place is not None and not np.allclose(q[-1],place['joint_q'][0],rtol=0,atol=1e-10):
        raise ValueError('Taught entry references do not join')
    root=Path(__file__).resolve().parents[3]
    sys.path.insert(0,str(root/'reproductions/ludan_greedy_20261023/deployment_ws/src/elfin_trajectory_executor'))
    from elfin_trajectory_executor.servo_j import fjt_to_servo_j_deg
    attach=np.flatnonzero(stages=='ESTIMATED_ATTACHMENT_FOR_PLANNING_ONLY')
    if len(attach)!=1:
        raise ValueError('Expected one attachment planning marker')
    contact=int(attach[0])-1
    a.output.mkdir(parents=True,exist_ok=False)
    operations=[]
    def motion(name,positions,times,velocities=None,labels=None):
        positions=np.asarray(positions);times=np.asarray(times)-times[0]
        points,error=fjt_to_servo_j_deg(positions.tolist(),times.tolist(),dt_s=.02)
        if error:raise ValueError(error)
        filename=name+'.json'
        payload=dict(period_s=.02,positions_deg=points)
        if labels is not None:
            payload['source_stages']=[str(s) for s in labels]
            payload['source_positions_deg']=np.degrees(positions).tolist()
        if velocities is not None:
            sample_times=np.minimum(np.arange(len(points))*.02,times[-1])
            velocities=np.asarray(velocities)
            payload['velocities_deg_s']=np.degrees(np.column_stack([
                np.interp(sample_times,times,velocities[:,axis]) for axis in range(6)])).tolist()
            payload['velocity_source']='analytic_source_curve'
        (a.output/filename).write_text(json.dumps(payload))
        operations.append(dict(type='servoj',name=name,reference=filename,
            duration_s=float(times[-1]),execution_completed=False))
    motion('observe_and_contact',q[:contact+1],t[:contact+1],None if qd is None else qd[:contact+1],stages[:contact+1])
    operations.append(dict(type='vacuum_on_and_confirm',completed=False,
        note='Requires actual valve operation and verified holding evidence; no simulated ACK'))
    motion('lift_and_entry',q[contact:],t[contact:],None if qd is None else qd[contact:],stages[contact:])
    if place is None:
        result=dict(scope='Pick portion only, no place or occupancy commit',operations=operations,
            source_pick=str(a.pick),motion_sent=False,io_written=False,hardware_cycle_completed=False)
        (a.output/'cycle.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(dict(output=str(a.output),scope=result['scope'])));return
    release=int(place['release_frame_index'])
    pq=np.asarray(place['joint_q']);pt=np.asarray(place['times'])
    profile=json.loads(Path(__file__).with_name('servoj_speed_profile.json').read_text())
    # Place-worker timing is separate from pick's phase-based retimer.
    pt=(pt-pt[0])*profile.get('place_reference_time_scale',1.0)
    operations.append(dict(type='bind_async_result',scene_version=place['scene_version'],
        completed=False,note='Bind current measured entry, attachment and committed scene'))
    motion('place',pq[:release+1],pt[:release+1])
    operations.append(dict(type='release_and_confirm_vacuum_off',completed=False,
        note='Actual release IO; released pose follows the planned-pose assumption'))
    motion('empty_return',pq[release:],pt[release:])
    operations.append(dict(type='commit_planned_occupancy',completed=False,
        predicted_occupied=place['predicted_occupied'],
        predicted_release_world_pose=place['predicted_release_world_pose'],
        note='User-authorized assumption: released pose equals planned pose, not observed settling'))
    prefix_catalog=place.get('committed_catalog',[])
    if len(prefix_catalog)!=request['source_index']:
        raise ValueError('Missing size catalog for the committed prefix')
    next_scene=dict(scene_version=str(request['scene_version'])+'-placed-'+str(request['source_index']+1),
        catalog=prefix_catalog+[observation['fitted']['dimensions_m']],
        occupied=request['occupied']+[place['predicted_occupied']],
        placed_world_poses=dict(request['placed_world_poses']),
        occupancy_source='planned_release_pose_assumption',
        released_pose_observed=False,physical_settling_verified=False)
    # Preserve the worker's B1/B2 prefix keys (also normalize legacy numeric keys).
    if set(next_scene['placed_world_poses'])=={str(i) for i in range(request['source_index'])}:
        next_scene['placed_world_poses']={f'B{int(k)+1}':v
            for k,v in next_scene['placed_world_poses'].items()}
    next_scene['placed_world_poses'][place['predicted_occupied']['id']]=place['predicted_release_world_pose']
    result=dict(scope=__doc__,operations=operations,source_pick=str(a.pick),source_place=str(a.place),
        motion_sent=False,io_written=False,hardware_cycle_completed=False,
        backend='Existing classmate ServoJ resampler; official SDK execution not started',
        source_place_compute_s=place['compute_s'],
        source_place_initialization_s=place['initialization_s'],
        reference_motion_s=sum(op.get('duration_s',0) for op in operations),
        actual_async_overlap_measured=False)
    result['next_scene_after_execution']=next_scene
    (a.output/'next_scene_preview.json').write_text(json.dumps(next_scene,indent=2))
    (a.output/'cycle.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
