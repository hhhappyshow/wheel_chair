"""S20 official SDK boundary. Import/default commands never actuate hardware."""
import argparse
from datetime import datetime,timezone
import importlib
import inspect
import json
import math
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
SDK_ROOT=Path('/home/hku_reconova/RoboticArm-master/third_party/huayan_python_sdk')
CONTROLLER='192.168.0.10'


class SDKInterfaces:
    """Official StartServo/PushServoJ + existing box IO, no new controller.

    Sending and IO methods are callable integration points only. The CLI below
    does not expose actuation or enable/reset. No connection occurs on import.
    """
    def __init__(self):
        sys.path.insert(0,str(SDK_ROOT))
        sdk=importlib.import_module('CPS')
        self.client=sdk.CPSClient()
        self.client.g_clients=[sdk.RbtClient() for _ in range(self.client.MaxBox)]
        self.client.g_plugin_clients=[sdk.PluginClient() for _ in range(self.client.MaxBox)]
        self.client.g_client_state=[False]*self.client.MaxBox
        self.client.g_plugin_client_state=[False]*self.client.MaxBox
        self.connected=False;self.servo_active=False
    def call(self,name,*args):
        code=getattr(self.client,name)(*args)
        if code:raise RuntimeError(f'{name} returned {code}')
    def __enter__(self):
        self.call('HRIF_Connect',0,CONTROLLER,10003);self.connected=True
        return self
    def __exit__(self,*args):
        try:
            if self.servo_active:self.stop_servoj()
        finally:
            if self.connected:
                self.client.HRIF_DisConnect(0);self.connected=False
                heartbeat=getattr(self.client.g_clients[0],'hb_thread',None)
                if heartbeat is not None:heartbeat.join(timeout=2.)
    def read(self,name,*args):
        values=[];self.call(name,0,*args,values);return values
    def snapshot(self):
        began=time.monotonic()
        q=list(map(float,self.read('HRIF_ReadActJointPos',0)[:6]))
        fsm=self.read('HRIF_ReadCurFSM',0)
        return dict(captured_at_client_utc=datetime.now(timezone.utc).isoformat(),
            monotonic_s=time.monotonic(),q_deg=q,q_rad=[math.radians(v) for v in q],
            qd_deg_s=list(map(float,self.read('HRIF_ReadActJointVel',0)[:6])),
            controller_version=self.read('HRIF_ReadVersion',0),
            robot_state=list(map(int,self.read('HRIF_ReadRobotState',0))),
            fsm=fsm,fsm_name=self.client.dic_FSM.get(int(fsm[0])),
            flange_pose_raw=self.read('HRIF_ReadActPos',0),
            tcp_config=self.read('HRIF_ReadCurTCP',0),ucs_config=self.read('HRIF_ReadCurUCS',0),
            joint_max_velocity_deg_s=list(map(float,self.read('HRIF_ReadJointMaxVel',0))),
            joint_max_acceleration_deg_s2=list(map(float,self.read('HRIF_ReadJointMaxAcc',0))),
            override=self.read('HRIF_ReadOverride',0),payload_readback=self.read('HRIF_ReadPayload'),
            vacuum=self.read_vacuum(),elapsed_s=time.monotonic()-began,
            motion_sent=False,actuator_outputs_written=False,controller_configuration_written=False)
    def read_vacuum(self):
        return dict(DI0=int(self.read('HRIF_ReadBoxDI',0)[0]),
                    DO0=int(self.read('HRIF_ReadBoxDO',0)[0]),
                    DO1=int(self.read('HRIF_ReadBoxDO',1)[0]))
    def start_servoj(self):
        self.call('HRIF_StartServo',0,0,.02,.2);self.servo_active=True
    def push_joint_positions_rad(self,q_rad):
        values=list(q_rad)
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            raise ValueError('Six finite joint positions in radians required')
        if not self.servo_active:raise RuntimeError('StartServo must precede PushServoJ')
        self.call('HRIF_PushServoJ',0,0,[math.degrees(v) for v in values])
    def stop_servoj(self):
        self.call('HRIF_GrpStop',0,0);self.servo_active=False
    def set_vacuum(self,on):
        if self.read_vacuum()['DO1']:
            raise RuntimeError('Release output is active; inspect actual valve wiring/state')
        self.call('HRIF_SetBoxDO',0,0,int(bool(on)))
        return self.read_vacuum()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    snap=sub.add_parser('snapshot');snap.add_argument('--output',type=Path)
    inverse=sub.add_parser('inverse-kinematics')
    inverse.add_argument('--request',type=Path,required=True)
    inverse.add_argument('--output',type=Path)
    sub.add_parser('contract')
    sub.add_parser('camera-status')
    sub.add_parser('camera-profiles')
    prep=sub.add_parser('prepare-reference')
    prep.add_argument('--input',type=Path,required=True);prep.add_argument('--output',type=Path,required=True)
    prep.add_argument('--phase',choices=('normal','approach','final_place'),required=True)
    args=parser.parse_args()
    if args.command=='snapshot':
        with SDKInterfaces() as sdk:result=sdk.snapshot()
    elif args.command=='inverse-kinematics':
        request=json.loads(args.request.read_text());rows=[]
        with SDKInterfaces() as sdk:
            for query in request['queries']:
                pose=query['flange_pose_mm_rpy_deg'];seed=query['seed_q_deg']
                if len(pose)!=6 or len(seed)!=6 or not all(math.isfinite(v) for v in pose+seed):
                    raise ValueError('IK requires six finite flange pose and joint seed values')
                values=[];began=time.monotonic()
                code=sdk.client.HRIF_GetInverseKin(0,0,pose,seed,[0.]*6,[0.]*6,values)
                description=[]
                if code:sdk.client.HRIF_GetErrorCodeStr(0,code,description)
                rows.append(dict(query,return_code=code,error_description=description,
                    result=values,elapsed_s=time.monotonic()-began))
        result=dict(queries=rows,sdk_method='HRIF_GetInverseKin',tcp_ucs=[0.]*6,
            motion_sent=False,actuator_outputs_written=False,controller_configuration_written=False)
    elif args.command=='contract':
        sys.path.insert(0,str(SDK_ROOT));module=importlib.import_module('CPS')
        methods=('HRIF_StartServo','HRIF_PushServoJ','HRIF_GrpStop','HRIF_ReadActJointPos',
            'HRIF_ReadRobotState','HRIF_ReadBoxDI','HRIF_ReadBoxDO','HRIF_SetBoxDO')
        result=dict(sdk_module=module.__file__,sdk_methods={n:str(inspect.signature(getattr(module.CPSClient,n))) for n in methods},
            speed_profile=json.loads((HERE/'servoj_speed_profile.json').read_text()),
            scene_config=str(HERE/'scene_tf.yaml'),waypoints=str(HERE/'pick_waypoints.json'),
            controller_connected=False,motion_sent=False,actuator_outputs_written=False,
            default_commands_read_only=True,servo_units='degrees at SDK boundary; planner uses radians',
            servo_period_s=.02,lookahead_s=.2,global_override_is_not_servoj_time_scaling=True)
    elif args.command in ('camera-status','camera-profiles'):
        binary='/home/hku_reconova/librealsense_dds/bin/rs-enumerate-devices'
        import os
        env=dict(os.environ);env['LD_LIBRARY_PATH']='/home/hku_reconova/librealsense_dds/lib:'+env.get('LD_LIBRARY_PATH','')
        command=[binary] if args.command=='camera-profiles' else [binary,'-s']
        proc=subprocess.run(command,env=env,capture_output=True,text=True,timeout=40)
        result=dict(camera_source='existing official DDS-enabled librealsense tool',
            exit_code=proc.returncode,stdout=proc.stdout,stderr=proc.stderr,
            device_stream_started=False,motion_sent=False,actuator_outputs_written=False)
    else:
        import numpy as np
        from servoj_reference import prepare_reference
        data=np.load(args.input)
        result=prepare_reference(data['times_s'],data['joint_q'],velocities_rad_s=data['joint_qd'],
            accelerations_rad_s2=data['joint_qdd'],phase=args.phase)
    output=getattr(args,'output',None)
    if output:
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
