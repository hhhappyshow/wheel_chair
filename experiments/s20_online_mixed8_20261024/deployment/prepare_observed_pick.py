"""Prepare the taught observation/contact/lift/entry chain from real input.

Only existing FK/IK/CAD planning runs here. Attachment after contact is an
observed-geometry hypothesis for loaded planning, never a hardware seal ACK.
No SDK connection, enable, pump output or physical motion is performed.
"""
import argparse
import json
from pathlib import Path
import sys
import time

HERE=Path(__file__).resolve().parent
EXPERIMENT=HERE.parent
ROOT=EXPERIMENT.parents[1]
sys.path[:0]=[str(EXPERIMENT),str(EXPERIMENT/'perception'),str(ROOT/'experiments/s20_greedy_async_20261023')]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--observation',type=Path,required=True)
    p.add_argument('--grasp',type=Path,required=True)
    p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--site-urdf',type=Path,required=True)
    p.add_argument('--waypoints',type=Path,default=HERE/'pick_waypoints.json')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    import numpy as np
    from scipy.spatial.transform import Rotation
    import newton
    from suitcase_capture import make_runtime
    from observed_collision import Collision
    from servoj_reference import prepare_reference
    from prepare_observed_place import serial
    from pick_reference_recorder import PickReferenceRecorder
    began=time.monotonic()
    observation=json.loads(a.observation.read_text());fitted=observation['fitted']
    grasp=json.loads(a.grasp.read_text())['selected'];snapshot=json.loads(a.snapshot.read_text())
    waypoints=json.loads(a.waypoints.read_text())
    r=make_runtime(a.output/'runtime',[fitted['dimensions_m']],site_urdf=a.site_urdf)
    r.collision=Collision(r.w)
    initialization_s=time.monotonic()-began
    box=np.array(fitted['T_world_box']);body=r.boxes[0]
    r.fixed={body:np.r_[box[:3,3],Rotation.from_matrix(box[:3,:3]).as_quat()]}
    r.collision.arrived={0};r.collision.current_source=0
    r.history.clear();r.qs.clear();r.stages.clear();r.simtime.clear();r.t=0.
    r.frame(np.array(snapshot['q_rad']),'MEASURED_START')
    recorder=PickReferenceRecorder(r,a.site_urdf)
    r.execute=recorder.execute
    r.frame=recorder.frame
    velocity_rows={}
    def retain_velocity(reference):
        start=len(r.qs)
        velocity_rows.update({start+i:np.radians(v) for i,v in
            enumerate(reference['velocities_deg_s'])})
    report=dict(scope=__doc__,success=False,motion_sent=False,io_written=False,
        snapshot=str(a.snapshot),observation=str(a.observation),site_urdf=str(a.site_urdf),
        stages=[],attachment_confirmed=False,observed_ground_used=True,
        ground_plane=fitted['ground_plane'],joint_limits_source='Existing selected live controller limits, unchanged')
    # Replace only the generic pickup ground with the actual observed plane.
    ground=np.array(fitted['ground_plane'],float);ground/=np.linalg.norm(ground[:3])
    normal=ground[:3]
    ground_rotation=Rotation.align_vectors([normal],[[0,0,1]])[0]
    transforms=r.model.shape_transform.numpy()
    index=r.model.shape_label.index('world_ground')
    half_height=r.model.shape_scale.numpy()[index,2]
    transforms[index]=np.r_[-ground[3]*normal-normal*half_height,ground_rotation.as_quat()]
    r.model.shape_transform.assign(transforms);r.collision.local[index]=transforms[index]
    # A tilted top-face OBB can intersect its observed support plane before
    # lift. Permit only that measured initial overlap, never a deeper one.
    half_extent=np.abs(box[:3,:3].T@normal)@(np.array(fitted['dimensions_m'])/2)
    r.collision.source_ground_reference_depth_m=max(0.,float(
        half_extent-normal@box[:3,3]-ground[3]))
    report['initial_source_ground_proxy_overlap_m']=r.collision.source_ground_reference_depth_m
    def retime(curve,stage):
        recorder.anchor_knots=curve(curve.x)
        phase='approach' if stage=='CONTACT' else 'normal'
        times=curve.x
        reference=prepare_reference(times,curve(times),velocities_rad_s=curve.derivative(1)(times),
            accelerations_rad_s2=curve.derivative(2)(times),phase=phase)
        retain_velocity(reference)
        report['stages'].append(dict(stage=stage,reference_duration_s=reference['duration_s'],
            source_duration_s=reference['source_duration_s'],phase=phase))
        return np.radians(reference['positions_deg']),reference['duration_s']
    def joint_target(name,stage):
        q0=r.q.copy();q1=np.array(waypoints[name]['q_rad']);zero=np.zeros((2,6))
        reference=prepare_reference([0,1],np.array([q0,q1]),velocities_rad_s=zero,
            accelerations_rad_s2=zero,phase='normal')
        retain_velocity(reference)
        grid=np.radians(reference['positions_deg'])
        recorder.anchor_knots=np.array([q0,q1])
        # Geometric validation is independent of the unchanged execution clock.
        r.execute(grid,stage)
        report['stages'].append(dict(stage=stage,reference_duration_s=reference['duration_s'],
            exact_taught_q_rad=q1.tolist(),phase='normal'))
    try:
        if not r.validate(r.q):raise RuntimeError('MEASURED_START: CAD '+str(r.collision.last_hits))
        joint_target('observation','OBSERVATION')
        contact=np.array(grasp['T_world_tcp'])
        geometric=np.array(grasp.get('T_world_tcp_geometric',grasp['T_world_tcp']))
        top=geometric[:3,3];quat=Rotation.from_matrix(contact[:3,:3]).as_quat()
        normal=-contact[:3,2]
        r.move([(top+normal*.10,quat)],'PRECONTACT',3.,reference_transform=retime)
        r.move([(contact[:3,3],quat)],'CONTACT',1.,reference_transform=retime)
        tcp=r.state.body_q.numpy()[r.tcp];rotation=Rotation.from_quat(tcp[3:])
        r.w.grip_offset=rotation.inv().apply(box[:3,3]-tcp[:3])
        r.w.grip_rot=rotation.inv()*Rotation.from_matrix(box[:3,:3]);r.w.held=body
        r.frame(r.q,'ESTIMATED_ATTACHMENT_FOR_PLANNING_ONLY')
        r.move([(top+[0,0,.20],quat)],'VERTICAL_CLEARANCE',2.,reference_transform=retime)
        joint_target('post_pick_lift','TAUGHT_LIFT')
        joint_target('container_entry','TAUGHT_ENTRY')
        report.update(success=True,end_q_rad=r.q.tolist(),
            T_tcp_box_estimate=grasp['T_tcp_box_estimated'],
            reference_duration_s=r.simtime[-1],actual_events_not_performed=['contact','vacuum_on','seal','lift','entry'])
    except Exception as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
        report['last_stage']=r.stages[-1]
        report['collision_pairs']=r.collision.last_hits
    finally:
        report['wall_s']=time.monotonic()-began
        report['preparation_backend']='sparse_geometric_CAD_batch_CPU_FK_dense_reference_unchanged'
        report['preparation_timing_s']=recorder.timing
        report['runtime_initialization_s']=initialization_s
        report['cartesian_generation_events']=r.events
        report['cad_validated_reference_points']=recorder.validated_points
        report['dense_reference_points']=recorder.reference_points
        report['geometric_sampling']=recorder.sampling
        np.savez_compressed(a.output/'reference.npz',joint_q=np.array(r.qs),times_s=np.array(r.simtime),
            joint_qd=np.array([velocity_rows.get(i,np.zeros(6)) for i in range(len(r.qs))]),
            stage=np.array(r.stages),body_q=np.array(r.history),body_labels=np.array(r.model.body_label))
        (a.output/'result.json').write_text(json.dumps(report,indent=2,default=serial))
    print(json.dumps(report,default=serial))


if __name__=='__main__':main()
