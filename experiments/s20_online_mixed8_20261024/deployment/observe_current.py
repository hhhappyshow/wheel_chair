"""One real current observation through existing capture, fit, grasp and prior.

No arm motion, enable or pump IO. Each invocation creates a new observation;
previous target coordinates are never loaded. Optional scene state contains
only previously placed boxes; without it the empty ULD assumption is explicit.
"""
import argparse
import contextlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
from current_profile import load_profile

HERE=Path(__file__).resolve().parent
EXPERIMENT=HERE.parent
ROOT=EXPERIMENT.parents[1]


def offset_contact(grasp,fitted,offset_z_m):
    if offset_z_m==0:return grasp
    import copy
    import numpy as np
    selected=copy.deepcopy(grasp['selected'])
    surface=np.array(selected['T_world_tcp'])
    contact=surface.copy();contact[2,3]+=offset_z_m
    selected.update(T_world_tcp_geometric=surface.tolist(),T_world_tcp=contact.tolist(),
        tcp_position_world_m=contact[:3,3].tolist(),contact_offset_world_m=[0.,0.,offset_z_m],
        T_tcp_box_estimated=(np.linalg.inv(contact)@np.array(fitted['T_world_box'])).tolist())
    return dict(grasp,selected=selected,contact_offset_basis='World vertical position offset; not force control')


def main(argv=None):
    profile = load_profile()
    perception = profile['perception']
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=perception['capture_deadline_s'])
    p.add_argument('--frames',type=int,default=perception['frames'])
    p.add_argument('--scene-state',type=Path)
    p.add_argument('--camera-session',type=Path)
    p.add_argument('--perception-target',choices=('luggage','any'),default='luggage')
    p.add_argument('--contact-offset-z-m',type=float,default=0.)
    a=p.parse_args(argv);a.output.mkdir(parents=True,exist_ok=False)
    began=time.monotonic()
    timing={}
    stage=time.monotonic()
    capture=['--output',a.output/'capture','--duration',a.duration,'--frames',a.frames,'--snapshot-joints',
        '--controller-snapshot-output',a.output/'controller_snapshot.json']
    if a.camera_session:capture+=['--camera-session',a.camera_session]
    from camera_receive import main as capture_current
    capture_current(list(map(str,capture)))
    timing['capture_and_snapshot_s']=time.monotonic()-stage
    stage=time.monotonic()
    from received_to_observation import main as convert_observation
    convert_observation(list(map(str,['--capture',a.output/'capture','--scene',HERE/'scene_tf.yaml',
        '--site-urdf',HERE/'site_robot.urdf','--weights',EXPERIMENT/perception['weights'],
        '--yolo-confidence',perception['confidence'],
        '--perception-target',a.perception_target,
        '--output',a.output/'observation'])))
    timing['conversion_s']=time.monotonic()-stage
    observation=json.loads((a.output/'observation/observation.json').read_text())
    if not observation.get('fitted'):
        result=dict(status=observation['status'],motion_sent=False,io_written=False,
            wall_s=time.monotonic()-began,timing_s=timing)
        (a.output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result));return 2
    stage=time.monotonic()
    spec=importlib.util.spec_from_file_location('site_fk',EXPERIMENT/'pose_readback/measured_pose.py')
    fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)
    robot=fk.site_empty_urdf(HERE/'site_robot.urdf',ROOT/'reproductions/ludan_greedy_20261023')
    ET.ElementTree(robot).write(a.output/'resolved_robot.urdf',encoding='unicode')
    sys.path.insert(0,str(EXPERIMENT/'perception'))
    from grasp_interface import EnvelopeGraspPlanner
    fitted=json.loads((a.output/'observation/observed_box.json').read_text())
    grasp=EnvelopeGraspPlanner(a.output/'resolved_robot.urdf').plan(fitted,
        preferred_fixture_direction_world=[1,0,0])
    grasp=offset_contact(grasp,fitted,a.contact_offset_z_m)
    (a.output/'grasp.json').write_text(json.dumps(grasp,indent=2))
    timing['grasp_s']=time.monotonic()-stage
    stage=time.monotonic()
    from planning_candidates import CandidatePlanner
    scene=json.loads(a.scene_state.read_text()) if a.scene_state else dict(occupied=[],scene_version=a.output.name)
    request=dict(request_id=a.output.name,scene_version=scene['scene_version'],
        observed_dimensions_m=observation['fitted']['dimensions_m'],committed_occupancy=scene['occupied'])
    candidates=CandidatePlanner(HERE/'scene_tf.yaml').plan(request)
    (a.output/'candidates.json').write_text(json.dumps(candidates,indent=2))
    timing['candidates_s']=time.monotonic()-stage
    result=dict(scope=__doc__,status='observed_geometry_ready',motion_sent=False,io_written=False,
        observation='observation/observation.json',grasp='grasp.json',candidates='candidates.json',
        scene=str(HERE/'scene_tf.yaml'),site_urdf=str(HERE/'site_robot.urdf'),
        empty_uld_assumption=not scene['occupied'],committed_count=len(scene['occupied']),future_arrivals_used=0,
        wall_s=time.monotonic()-began,timing_s=timing,trajectory_or_hardware_execution_completed=False)
    (a.output/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return 0


def serve():
    # One session-owned process reuses model imports; every request captures new RGB-D.
    with contextlib.redirect_stdout(sys.stderr):
        from received_to_observation import load_model
        import numpy as np
        model=load_model(EXPERIMENT/load_profile()['perception']['weights'])
        from yolo_sim_perception import infer
        # Warm only the predictor, before observation return; no target is retained.
        infer(model,np.zeros((360,640,3),dtype=np.uint8),np.full((360,640,3),np.nan),device='cpu')
    for line in sys.stdin:
        try:
            request=json.loads(line)
            with contextlib.redirect_stdout(sys.stderr):code=main(request['argv'])
            result=dict(returncode=code)
        except Exception as exc:
            result=dict(returncode=1,error=f'{type(exc).__name__}: {exc}')
        print(json.dumps(result),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(serve() if sys.argv[1:]==['--serve'] else main())
