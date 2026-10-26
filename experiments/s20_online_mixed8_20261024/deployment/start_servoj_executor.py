"""Start the existing author ServoJ server, idle until it receives an action.

Run in the sourced site ROS environment. No trajectory or vacuum request is
sent here. The author's normal real-backend connection code is unchanged.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--servo-period-s',type=float,choices=(.02,.01),
        help='Optional interpolated sending period; does not shorten trajectory time')
    a=p.parse_args()
    from jetson_interfaces import SDKInterfaces
    with SDKInterfaces() as sdk:
        snapshot=sdk.snapshot()
    if snapshot['fsm_name']!='StandBy':
        raise RuntimeError('Controller is not currently StandBy; no executor started')
    root=Path(__file__).resolve().parents[3]
    source=root/'reproductions/ludan_greedy_20261023/deployment_ws/src/elfin_trajectory_executor'
    env=dict(os.environ)
    env['PYTHONPATH']=str(source)+os.pathsep+env.get('PYTHONPATH','')
    env['HUAYAN_SDK']='/home/hku_reconova/RoboticArm-master/third_party/huayan_python_sdk'
    a.output.mkdir(parents=True,exist_ok=False)
    profile=json.loads(Path(__file__).with_name('servoj_speed_profile.json').read_text())
    fraction=profile.get('executor_limit_fraction',profile['phase_velocity_fractions']['normal'])
    period=a.servo_period_s if a.servo_period_s is not None else profile['servo_period_s']
    command=[sys.executable,str(Path(__file__).with_name('run_servoj_executor.py')),
        '--ros-args','-p','mode:=real','-p','execution_backend:=servo_j',
        '-p',f'controller_limit_fraction:={fraction}',
        '-p',f'max_velocity_deg:={profile.get("executor_max_velocity_deg_s",60.0)}',
        '-p',f'servo_j_servo_time:={period}',
        '-p',f'servo_j_lookahead_time:={profile["lookahead_s"]}',
        '-p','power_off_on_disconnect:=false']
    with (a.output/'executor.log').open('x') as log:
        proc=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result=dict(pid=proc.pid,command=command,snapshot_before_start=snapshot,
        motion_sent=False,io_written=False,
        note='Server launch only, readiness must come from actual ROS parameters and feedback')
    (a.output/'launch.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(pid=proc.pid,output=str(a.output),motion_sent=False,io_written=False)))


if __name__=='__main__':main()
