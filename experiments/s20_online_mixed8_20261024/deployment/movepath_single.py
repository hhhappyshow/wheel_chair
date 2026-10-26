"""One explicitly authorized empty-arm MovePathJ comparison; no enable or IO."""
import argparse
import json
from pathlib import Path
import time

from jetson_interfaces import SDKInterfaces


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--empty-confirmed',action='store_true')
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    report=dict(motion_sent=False,io_written=False,completed=False)
    events=[]
    def event(name,**data):
        events.append(dict(event=name,monotonic_s=time.monotonic(),**data))
        (a.output/'events.json').write_text(json.dumps(events,indent=2))
    with SDKInterfaces() as sdk:
        try:
            snap=sdk.snapshot();report['before']=snap
            if snap['fsm_name']!='StandBy':raise RuntimeError('Controller is not StandBy')
            if not a.empty_confirmed or snap['vacuum']['DI0']:
                raise RuntimeError('This comparison requires a confirmed empty tool')
            start=snap['q_deg'];goal=start.copy();goal[0]+=1.
            points=[[x+(y-x)*k/4 for x,y in zip(start,goal)] for k in range(5)]
            ratio=min(12./max(snap['joint_max_velocity_deg_s']),
                10./max(snap['joint_max_acceleration_deg_s2']))
            report.update(target_q_deg=goal,points_deg=points,speed_ratio=ratio,
                speed_ratio_is_not_a_measured_velocity=True,radius_mm=1.)
            if not a.execute:
                report['status']='prepared_only';return
            name='cmp_'+str(time.monotonic_ns())
            sdk.call('HRIF_StartPushMovePathJ',0,0,name,ratio,1.)
            for point in points:sdk.call('HRIF_PushMovePathJ',0,0,name,point)
            sdk.call('HRIF_EndPushMovePathJ',0,0,name)
            began=time.monotonic()
            while True:
                state=sdk.read('HRIF_ReadPathState',0,name)
                event('path_state',value=state)
                if int(state[0])==3 and int(state[1])==0:break
                if int(state[0])==5 or int(state[1]):raise RuntimeError('Path calculation failed: '+str(state))
                if time.monotonic()-began>30:raise RuntimeError('Path calculation did not finish')
                time.sleep(.1)
            report['path_info']=sdk.read('HRIF_ReadPathInfo',0,name)
            live=sdk.snapshot()
            if live['fsm_name']!='StandBy' or max(abs(x-y) for x,y in zip(live['q_deg'],start))>.05:
                raise RuntimeError('Start changed during preparation')
            event('move_path_command',path_name=name)
            report['motion_sent']=True
            sdk.call('HRIF_MovePathJ',0,0,name)
            began=time.monotonic()
            while True:
                q=list(map(float,sdk.read('HRIF_ReadActJointPos',0)[:6]))
                fsm=int(sdk.read('HRIF_ReadCurFSM',0)[0])
                event('motion_feedback',q_deg=q,fsm=fsm)
                if fsm in (21,22,24):raise RuntimeError('Controller stopped: '+str(fsm))
                if fsm==33 and max(abs(x-y) for x,y in zip(q,goal))<.05:
                    report.update(completed=True,motion_wall_s=time.monotonic()-began);break
                if time.monotonic()-began>60:raise RuntimeError('Motion did not finish')
                time.sleep(.05)
            report['after']=sdk.snapshot()
        except Exception as exc:
            report['error']=f'{type(exc).__name__}: {exc}'
            if report['motion_sent']:
                report['stop_return_code']=sdk.client.HRIF_GrpStop(0,0)
        finally:
            (a.output/'result.json').write_text(json.dumps(report,indent=2))
            print(json.dumps(report))


if __name__=='__main__':main()
