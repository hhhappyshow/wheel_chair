"""Prepare an online place/empty-return reference from the current real box.

Reuses the existing Contact V2 worker and classmate ServoJ resampler. This
does not initialize the real controller, operate vacuum, or commit occupancy.
The taught entry is a future start assumption, not an executed transfer.
"""
import argparse
import json
from pathlib import Path
import sys
import time
from current_profile import load_profile

HERE=Path(__file__).resolve().parent
EXPERIMENT=HERE.parent
ROOT=EXPERIMENT.parents[1]
sys.path[:0]=[str(EXPERIMENT),str(EXPERIMENT/'perception'),str(ROOT/'experiments/s20_greedy_async_20261023')]


def serial(value):
    import numpy as np
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--observation',type=Path,required=True)
    p.add_argument('--grasp',type=Path,required=True)
    p.add_argument('--site-urdf',type=Path,required=True)
    p.add_argument('--waypoints',type=Path,default=HERE/'pick_waypoints.json')
    p.add_argument('--scene-state',type=Path,
        help='Measured committed prefix: catalog, occupied, placed_world_poses, scene_version')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    import numpy as np
    from scipy.spatial.transform import Rotation
    from straight_place.async_sparse_place import initialize_worker,plan_in_worker
    began=time.monotonic()
    observation=json.loads(a.observation.read_text());fitted=dict(observation['fitted'])
    packing = load_profile()['packing']
    padding_xy_m = packing['padding_xy_per_side_m']
    dimensions = np.asarray(fitted['dimensions_m'])+np.array([2*padding_xy_m, 2*padding_xy_m, 0.])
    fitted.update(planning_dimensions_m=dimensions.tolist(), padding_xy_per_side_m=padding_xy_m,
        planning_size_source='Measured box plus 5cm on each XY side; measured height and grasp unchanged')
    grasp=json.loads(a.grasp.read_text())['selected']
    t=np.array(grasp['T_world_tcp']);contact=np.r_[t[:3,3],Rotation.from_matrix(t[:3,:3]).as_quat()]
    waypoints=json.loads(a.waypoints.read_text())
    scene=json.loads(a.scene_state.read_text()) if a.scene_state else dict(
        catalog=[],occupied=[],placed_world_poses={},scene_version='current-site-empty-uld-20261025')
    source_index=len(scene['occupied'])
    if len(scene['catalog'])!=source_index:
        raise ValueError('Scene catalog must contain exactly the measured committed prefix')
    options=dict(packing['worker_options'],physical_padding_xy_m=padding_xy_m,
        max_box_top_m=packing['max_box_top_m'],
        site_urdf=str(a.site_urdf.resolve()))
    initialize_worker(a.output/'worker',scene['catalog']+[fitted['planning_dimensions_m']],waypoints,options)
    initialized=time.monotonic()
    request=dict(request_id=a.output.name,scene_version=scene['scene_version'],
        source_index=source_index,fitted=fitted,planned_contact_tcp=contact.tolist(),
        entry_q=waypoints['container_entry']['q_rad'],occupied=scene['occupied'],
        placed_world_poses=scene['placed_world_poses'])
    result=plan_in_worker(request)
    result.update(scope=__doc__,observation_source=str(a.observation),site_urdf=str(a.site_urdf),
        committed_catalog=scene['catalog'],
        initialization_s=initialized-began,total_wall_s=time.monotonic()-began,
        motion_sent=False,io_written=False,actual_attachment_confirmed=False,
        taught_entry_executed=False,occupancy_committed=False)
    (a.output/'planning_result.json').write_text(json.dumps(result,indent=2,default=serial))
    if result['success']:
        sys.path.insert(0,str(ROOT/'reproductions/ludan_greedy_20261023/deployment_ws/src/elfin_trajectory_executor'))
        from elfin_trajectory_executor.servo_j import fjt_to_servo_j_deg
        points,error=fjt_to_servo_j_deg(result['joint_q'].tolist(),result['times'].tolist(),dt_s=.02)
        if error:raise RuntimeError(error)
        result['sdk_reference']=dict(backend='official classic ServoJ',period_s=.02,
            positions_deg=points,source='existing classmate fjt_to_servo_j_deg',
            controller_contacted=False,execution_enabled=False,
            release_frame_index=result['release_frame_index'],
            release_note='Planner marker only; actual support/release event must be handled by hardware state machine')
        np.savez_compressed(a.output/'reference.npz',joint_q=result['joint_q'],times_s=result['times'],
            stage=np.array(result['stages']),positions_deg=np.asarray(points))
    (a.output/'request.json').write_text(json.dumps(request,indent=2,default=serial))
    (a.output/'result.json').write_text(json.dumps(result,indent=2,default=serial))
    print(json.dumps(dict(output=str(a.output),success=result['success'],error=result.get('error'),
        compute_s=result['compute_s'],total_wall_s=result['total_wall_s'],motion_sent=False)))


if __name__=='__main__':main()
