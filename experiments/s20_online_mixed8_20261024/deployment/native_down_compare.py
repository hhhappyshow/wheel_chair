"""Authorized empty-tool 200 mm base-Z descent using official MoveL."""
import argparse
import json
from pathlib import Path
import time

from jetson_interfaces import SDKInterfaces


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plan',type=Path)
    p.add_argument('--execute',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    report=dict(motion_sent=False,io_written=False,completed=False)
    rows=[]
    with SDKInterfaces() as sdk:
        try:
            snap=sdk.snapshot();report['before']=snap
            if snap['fsm_name']!='StandBy' or snap['vacuum']['DI0']:
                raise RuntimeError('Requires StandBy and the previously confirmed empty tool')
            if a.plan:
                plan=json.loads(a.plan.read_text())
            else:
                tcp=sdk.read('HRIF_ReadTCPByName',0,'TCP')
                ucs=sdk.read('HRIF_ReadUCSByName',0,'Base')
                if any(abs(float(v))>1e-6 for v in ucs+tcp+snap['tcp_config']+snap['ucs_config']):
                    raise RuntimeError('This comparison expects zero named/current TCP and UCS')
                pose=list(map(float,snap['flange_pose_raw'][6:12]));seed=snap['q_deg']
                if len(pose)!=6:raise RuntimeError('ActCoord reply has no six-dimensional pose')
                targets=[]
                for i in range(11):
                    target=pose.copy();target[2]-=20.*i
                    values=[]
                    sdk.call('HRIF_GetInverseKin',0,0,target,seed,list(map(float,tcp)),list(map(float,ucs)),values)
                    seed=list(map(float,values[:6]));targets.append(seed)
                goal=pose.copy();goal[2]-=200.
                plan=dict(start_q_deg=snap['q_deg'],start_pose_mm_deg=pose,target_pose_mm_deg=goal,
                    tcp_name='TCP',ucs_name='Base',tcp=tcp,ucs=ucs,
                    ik_q_deg=targets,speed_mm_s=30.,acceleration_mm_s2=50.,radius_mm=0.)
                (a.output/'plan.json').write_text(json.dumps(plan,indent=2))
            report['plan']=plan
            if not a.execute:return
            if max(abs(x-y) for x,y in zip(snap['q_deg'],plan['start_q_deg']))>.05:
                raise RuntimeError('Robot start changed since the prepared descent')
            for method,name,key in (('HRIF_ReadTCPByName','TCP','tcp'),('HRIF_ReadUCSByName','Base','ucs')):
                live=list(map(float,sdk.read(method,0,name)))
                if live!=list(map(float,plan[key])):raise RuntimeError('Named coordinate configuration changed')
            began=time.monotonic();report['motion_sent']=True
            sdk.call('HRIF_MoveL',0,0,plan['target_pose_mm_deg'],plan['ik_q_deg'][-1],
                'TCP','Base',plan['speed_mm_s'],plan['acceleration_mm_s2'],0.,0,0,0,'down20compare')
            while True:
                raw=sdk.read('HRIF_ReadActPos',0)
                fsm=int(sdk.read('HRIF_ReadCurFSM',0)[0])
                pose=list(map(float,raw[6:12]))
                rows.append(dict(monotonic_s=time.monotonic(),pose_mm_deg=pose,fsm=fsm))
                if fsm in (21,22,24):raise RuntimeError('Controller stopped: '+str(fsm))
                if fsm==33 and max(abs(x-y) for x,y in zip(pose[:3],plan['target_pose_mm_deg'][:3]))<.5:
                    report.update(completed=True,motion_wall_s=time.monotonic()-began);break
                if time.monotonic()-began>30:raise RuntimeError('Descent did not finish')
                time.sleep(.05)
            report['after']=sdk.snapshot()
        except Exception as exc:
            report['error']=f'{type(exc).__name__}: {exc}'
            if report['motion_sent']:report['stop_return_code']=sdk.client.HRIF_GrpStop(0,0)
        finally:
            (a.output/'feedback.json').write_text(json.dumps(rows,indent=2))
            (a.output/'result.json').write_text(json.dumps(report,indent=2))
            print(json.dumps(report))


if __name__=='__main__':main()
