"""Execute current online plans with controller-native paths, not ServoJ ticks.

Contact and vertical clearance use MoveL. Other accepted joint paths are
uploaded to MovePathJ. Native interpolation differs from the source curve.
"""
import argparse
import json
import math
from pathlib import Path
import time

from jetson_interfaces import SDKInterfaces


def distinct_joint_points(points):
    result=[]
    for point in points:
        if not result or point!=result[-1]:result.append(point)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cycle',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--action')  # Same session dispatcher as the ServoJ variant.
    p.add_argument('--place-only',action='store_true')
    p.add_argument('--initial-holding',action='store_true')
    p.add_argument('--vacuum-feedback',choices=('digital','operator'),default='digital')
    p.add_argument('--speed-scale',type=float,default=1.)
    a=p.parse_args();recipe=json.loads(a.cycle.read_text())
    if not math.isfinite(a.speed_scale) or a.speed_scale<=0:
        raise ValueError('speed-scale must be positive and finite')
    if not a.execute:
        print(json.dumps(dict(operations=recipe['operations'],motion_sent=False)));return
    a.output.mkdir(parents=True,exist_ok=False)
    log=(a.output/'events.jsonl').open('x')
    report=dict(completed=False,motion_requested=False,io_requested=False,
        backend='native_MoveL_MovePathJ',occupancy_committed=False,
        speed_scale=a.speed_scale,
        vacuum_confirmation_source=a.vacuum_feedback,
        pressure_sensor_verified=False,
        automatic_holding_loss_monitoring=a.vacuum_feedback=='digital',
        native_interpolation_is_source_curve=False,settled_placement_observed=False)
    holding=False;moving=False
    def record(event,**data):
        log.write(json.dumps(dict(event=event,monotonic_s=time.monotonic(),**data))+'\n');log.flush()
    def confirm(token,message):
        record('operator_confirmation_requested',token=token,message=message)
        if input(message+' Type '+token+' to continue, anything else stops: ').strip()!=token:
            raise RuntimeError('Operator did not confirm; no following motion sent')
        record('operator_confirmed',token=token,pressure_measured=False)
    def error_if_fault(sdk):
        fsm=int(sdk.read('HRIF_ReadCurFSM',0)[0])
        if fsm in (21,22,24):raise RuntimeError('Controller stopped: '+str(fsm))
        return fsm
    with SDKInterfaces() as sdk:
        def valve(bit,value):
            report['io_requested']=True
            sdk.call('HRIF_SetBoxDO',0,bit,int(value))
            record('valve_result',bit=bit,on=value)
        def wait_vacuum(value):
            if a.vacuum_feedback=='operator':
                if value:
                    began=time.monotonic()
                    while True:
                        io=sdk.read_vacuum()
                        if io['DO0']==1 and io['DO1']==0:break
                        if time.monotonic()-began>=2.:
                            raise RuntimeError('Suction output readback did not reach DO0=1/DO1=0: '+str(io))
                        time.sleep(.05)
                    record('suction_output_confirmed',value=io,
                        readback_wait_s=time.monotonic()-began,pressure_measured=False)
                    confirm('2','No vacuum sensor: confirm actual suction and supervise the following motion.')
                else:
                    confirm('3','Confirm the box is supported and the suction cup has physically released.')
                return
            began=time.monotonic();stable=None;last=None
            while time.monotonic()-began<30:
                io=sdk.read_vacuum();now=time.monotonic()
                if io!=last:record('vacuum_io_changed',value=io);last=io
                matched=io['DI0']==value and (not value or io['DO0']==1 and io['DO1']==0)
                if matched:
                    if stable is None:stable=now
                    if now-stable>=(1. if value else 0.):
                        record('vacuum_feedback',value=io,stable_s=now-stable);return
                else:stable=None
                time.sleep(.05)
            raise RuntimeError('Vacuum feedback did not confirm; no following motion sent')
        def wait_motion(goal):
            nonlocal moving
            began=time.monotonic()
            while time.monotonic()-began<180:
                q=list(map(float,sdk.read('HRIF_ReadActJointPos',0)[:6]))
                fsm=error_if_fault(sdk)
                if holding and a.vacuum_feedback=='digital':
                    io=sdk.read_vacuum()
                    if io['DI0']!=1:raise RuntimeError('Holding feedback lost; vacuum retained: '+str(io))
                if fsm==33 and max(abs(x-y) for x,y in zip(q,goal))<.05:
                    moving=False;record('trajectory_result',completed=True,q_deg=q,wall_s=time.monotonic()-began);return
                time.sleep(.05)
            raise RuntimeError('Native motion did not finish')
        def motion(points,linear=False):
            nonlocal moving
            current=list(map(float,sdk.read('HRIF_ReadActJointPos',0)[:6]))
            if max(abs(x-y) for x,y in zip(current,points[0]))>.05:
                raise RuntimeError('Native path start differs from actual joints; replan required')
            if len(distinct_joint_points(points))==1:
                record('native_segment_skipped',reason='identical_reference_points',source_points=len(points))
                return
            if max(abs(x-y) for x,y in zip(current,points[-1]))<.01 and len(points)<5:return
            if error_if_fault(sdk)!=33:raise RuntimeError('Native segment requires StandBy')
            if linear:
                pose=[]
                sdk.call('HRIF_GetForwardKin',0,0,points[-1],[0.]*6,[0.]*6,pose)
                record('native_move_command',method='MoveL',target_pose=pose,
                    velocity_mm_s=30.*a.speed_scale,acceleration_mm_s2=50.*a.speed_scale)
                report['motion_requested']=True;moving=True
                sdk.call('HRIF_MoveL',0,0,list(map(float,pose[:6])),points[-1],
                    'TCP','Base',30.*a.speed_scale,50.*a.speed_scale,0.,0,0,0,'native_contact')
            else:
                name='online_'+str(time.monotonic_ns())
                chosen=points[::5]
                if chosen[-1]!=points[-1]:chosen.append(points[-1])
                chosen=distinct_joint_points(chosen)
                if len(chosen)<5:chosen=distinct_joint_points(points)
                while len(chosen)<5:
                    k=max(range(len(chosen)-1),key=lambda k:sum((a-b)**2 for a,b in zip(chosen[k],chosen[k+1])))
                    chosen.insert(k+1,[(a+b)/2 for a,b in zip(chosen[k],chosen[k+1])])
                ratio=min(1.,a.speed_scale*min(12./max(report['before']['joint_max_velocity_deg_s']),
                    10./max(report['before']['joint_max_acceleration_deg_s2'])))
                sdk.call('HRIF_StartPushMovePathJ',0,0,name,ratio,1.)
                for q in chosen:sdk.call('HRIF_PushMovePathJ',0,0,name,q)
                sdk.call('HRIF_EndPushMovePathJ',0,0,name)
                began=time.monotonic()
                while time.monotonic()-began<30:
                    state=sdk.read('HRIF_ReadPathState',0,name)
                    if int(state[0])==3 and not int(state[1]):break
                    if int(state[0])==5 or int(state[1]):raise RuntimeError('Native path calculation failed: '+str(state))
                    time.sleep(.05)
                else:raise RuntimeError('Native path calculation timeout')
                record('native_move_command',method='MovePathJ',path_name=name,source_points=len(points),
                    uploaded_points=len(chosen),speed_ratio=ratio,
                    ratio_is_not_a_measured_velocity_or_acceleration=True,
                    path_info=sdk.read('HRIF_ReadPathInfo',0,name))
                report['motion_requested']=True;moving=True
                sdk.call('HRIF_MovePathJ',0,0,name)
            wait_motion(points[-1])
        try:
            report['before']=sdk.snapshot()
            if report['before']['fsm_name']!='StandBy':raise RuntimeError('Controller is not StandBy')
            for method,name in (('HRIF_ReadTCPByName','TCP'),('HRIF_ReadUCSByName','Base')):
                if any(float(v)!=0. for v in sdk.read(method,0,name)):
                    raise RuntimeError('Native mode requires the verified zero TCP/Base definitions')
            if any(float(v)!=0. for key in ('tcp_config','ucs_config') for v in report['before'][key]):
                raise RuntimeError('Controller current coordinates differ from verified native comparison')
            if not a.initial_holding and a.vacuum_feedback=='operator':
                confirm('1','Confirm the tool is empty and separated from the supported box before motion.')
            if a.vacuum_feedback=='digital' and recipe.get('role')=='empty_observation_return' and report['before']['vacuum']['DI0']:
                raise RuntimeError('Empty return cannot start with holding feedback')
            if a.initial_holding:wait_vacuum(1);holding=True
            operations=recipe['operations']
            if a.place_only:
                if not a.initial_holding:raise ValueError('place-only requires initial-holding')
                operations=operations[next(i for i,o in enumerate(operations) if o['type']=='bind_async_result'):]
            for op in operations:
                kind=op['type'];record('operation_start',operation=op)
                if kind=='servoj':
                    ref=json.loads((a.cycle.parent/op['reference']).read_text())
                    points=ref.get('source_positions_deg',ref['positions_deg'])
                    labels=ref.get('source_stages')
                    start=0
                    if labels is not None:
                        i=0
                        while i<len(labels):
                            j=i+1
                            while j<len(labels) and labels[j]==labels[i]:j+=1
                            if (labels[i] in ('CONTACT','VERTICAL_CLEARANCE') or labels[i].endswith(
                                    ('_EMPTY_SEPARATE','_INSERT_DEPTH','_SHIFT_LATERAL','_LOWER_FINAL'))) and j-i>1:
                                if i>start:motion(points[start:i])
                                motion(points[max(0,i-1):j],linear=True);start=j-1
                            i=j
                    if start<len(points)-1:motion(points[start:])
                elif kind=='vacuum_on_and_confirm':
                    valve(1,False);valve(0,True);wait_vacuum(1);holding=True
                elif kind=='bind_async_result':record('entry_feedback',scene_version=op['scene_version'])
                elif kind in ('release_and_confirm_support','release_and_confirm_vacuum_off'):
                    holding=False;valve(0,False);valve(1,True)
                    if a.vacuum_feedback=='operator':
                        time.sleep(.2);valve(1,False);wait_vacuum(0)
                    else:wait_vacuum(0);valve(1,False)
                elif kind=='commit_planned_occupancy':
                    (a.output/'scene_state.json').write_text(json.dumps(recipe['next_scene_after_execution'],indent=2))
                    report.update(occupancy_committed=True,occupancy_source='planned_release_pose_assumption')
                else:raise ValueError('Unknown operation '+kind)
                record('operation_end',type=kind)
            report.update(completed=True,motion_and_io_sequence_finished=True)
        except BaseException as exc:
            report['error']=f'{type(exc).__name__}: {exc}';record('stopped',error=report['error'])
            if moving:report['stop_return_code']=sdk.client.HRIF_GrpStop(0,0)
        finally:
            (a.output/'result.json').write_text(json.dumps(report,indent=2));log.close()
            if a.vacuum_feedback=='operator':
                print(json.dumps(dict(completed=report['completed'],error=report.get('error'),
                    confirmation_source='operator_without_pressure_sensor',output=str(a.output))))
            else:print(json.dumps(report))
    return 0 if report['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
