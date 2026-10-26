"""Consume a prepared cycle through the existing real ServoJ ROS executor.

Default only describes the recipe. Explicit execution needs an already-running
real/servo_j executor; this client never launches, enables or resets it. A
finished trajectory and DI transition do not certify a settled box pose.
"""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cycle',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--check-interfaces',action='store_true',help='Read existing ROS interfaces without sending goals or IO')
    p.add_argument('--executor-node',default='/trajectory_executor')
    p.add_argument('--action',required=True)
    p.add_argument('--event-timeout',type=float,default=30.)
    p.add_argument('--seal-stable-s',type=float,default=1.0,
        help='Require continuous fresh holding feedback before lift; not a fixed post-contact sleep')
    p.add_argument('--initial-holding',action='store_true',
        help='Continue a successful pick at entry; require fresh holding DI before motion')
    p.add_argument('--place-only',action='store_true',help='Continue after the pick portion already executed')
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    recipe=json.loads(a.cycle.read_text())
    if not a.execute and not a.check_interfaces:
        print(json.dumps(dict(scope=__doc__,operations=recipe['operations'],
            motion_sent=False,io_written=False),indent=2));return
    if a.output is None or a.output.exists():
        raise ValueError('Execution requires a new --output directory')
    if not math.isfinite(a.event_timeout) or a.event_timeout<=0:
        raise ValueError('event-timeout must be a positive finite duration')
    if not math.isfinite(a.seal_stable_s) or a.seal_stable_s<0:
        raise ValueError('seal-stable-s must be finite and nonnegative')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from trajectory_msgs.msg import JointTrajectoryPoint
    rclpy.init();node=Node('s20_observed_cycle_bridge')
    a.output.mkdir(parents=True)
    log=(a.output/'events.jsonl').open('x')
    report=dict(motion_requested=False,io_requested=False,completed=False,
        seal_required_stable_s=a.seal_stable_s,
        occupancy_committed=False,settled_placement_observed=False,
        recipe=str(a.cycle.resolve()))
    active=None;holding=False;io=None;io_seq=0;latest_q=None
    joint_names=[f'elfin_joint{i}' for i in range(1,7)]
    def record(event,**values):
        row=dict(event=event,monotonic_s=time.monotonic(),**values)
        log.write(json.dumps(row)+'\n');log.flush()
    def received_io(msg):
        nonlocal io,io_seq
        row=json.loads(msg.data)
        if row.get('schema')=='elfin_vacuum_io/v1':
            if io is None or any(row.get(k)!=io.get(k) for k in ('source','DI0','DO0','DO1')):
                record('vacuum_io_changed',value=row)
            io=row;io_seq+=1
    def received_joints(msg):
        nonlocal latest_q
        values=dict(zip(msg.name,msg.position))
        if all(name in values for name in joint_names):
            latest_q=dict(q_rad=[values[name] for name in joint_names],
                received_monotonic_s=time.monotonic())
    node.create_subscription(String,'/vacuum/io',received_io,10)
    node.create_subscription(JointState,'/joint_states',received_joints,10)
    def await_future(future):
        while rclpy.ok() and not future.done():rclpy.spin_once(node,timeout_sec=.02)
        if not future.done():raise RuntimeError('ROS shutdown')
        return future.result()
    def wait_io(expected,after):
        deadline=time.monotonic()+a.event_timeout
        stable_s=a.seal_stable_s if expected==1 else 0.
        stable_since=None;seen_seq=after
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if io_seq<=seen_seq or io is None:continue
            seen_seq=io_seq
            matches=io['DI0']==expected
            if expected==1:matches=matches and io['DO0']==1 and io['DO1']==0
            if not matches:
                if stable_since is not None:record('seal_confirmation_reset',value=dict(io))
                stable_since=None
                continue
            now=time.monotonic()
            if stable_since is None:stable_since=now
            if now-stable_since>=stable_s:
                record('vacuum_feedback',value=dict(io),stable_s=now-stable_since,
                    required_stable_s=stable_s);return
        raise RuntimeError(f'Vacuum feedback did not confirm DI0={expected} stably for {stable_s:.2f}s; '
            'no following motion sent')
    def valve(bit,on):
        client=node.create_client(SetBool,f'/elfin/vacuum/set_do{bit}')
        try:
            if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('Vacuum service unavailable')
            report['io_requested']=True
            result=await_future(client.call_async(SetBool.Request(data=on)))
            record('valve_result',bit=bit,on=on,success=result.success,message=result.message)
            if not result.success:raise RuntimeError(result.message)
        finally:node.destroy_client(client)
    try:
        params=node.create_client(GetParameters,a.executor_node.rstrip('/')+'/get_parameters')
        if not params.wait_for_service(timeout_sec=3.):raise RuntimeError('Existing executor unavailable')
        values=await_future(params.call_async(GetParameters.Request(names=[
            'mode','execution_backend','controller_limit_fraction','servoj_timing_policy','max_velocity_deg']))).values
        if [v.string_value for v in values[:2]]!=['real','servo_j']:
            raise RuntimeError('Executor must already be configured real + servo_j')
        profile=json.loads(Path(__file__).with_name('servoj_speed_profile.json').read_text())
        expected_fraction=profile.get('executor_limit_fraction',profile['phase_velocity_fractions']['normal'])
        if values[2].type!=3 or abs(values[2].double_value-expected_fraction)>1e-9:
            raise RuntimeError(f'Running executor speed is not {expected_fraction:.0%}; restart with current configuration')
        if values[3].string_value!='preserve_input_or_slow_v2':
            raise RuntimeError('Running executor has old timing policy; restart with current adapter')
        if 'executor_max_velocity_deg_s' in profile and (
                values[4].type!=3 or abs(values[4].double_value-profile['executor_max_velocity_deg_s'])>1e-9):
            raise RuntimeError('Running executor velocity base differs from reference configuration')
        action=ActionClient(node,FollowJointTrajectory,a.action)
        if not action.wait_for_server(timeout_sec=3.):raise RuntimeError('Trajectory action unavailable')
        if a.check_interfaces:
            for bit in (0,1):
                client=node.create_client(SetBool,f'/elfin/vacuum/set_do{bit}')
                try:
                    if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('Vacuum service unavailable')
                finally:node.destroy_client(client)
            deadline=time.monotonic()+a.event_timeout
            while (io is None or latest_q is None) and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
            if io is None or latest_q is None:raise RuntimeError('Joint or vacuum feedback unavailable')
            report.update(interfaces_available=True,executor_mode='real',execution_backend='servo_j',
                vacuum_feedback=io,joint_feedback=latest_q,
                note='No goal or valve command sent; not a physical cycle acceptance')
            print(json.dumps(report,indent=2));return
        if recipe.get('role')=='empty_observation_return':
            deadline=time.monotonic()+a.event_timeout
            while io is None and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
            if io is None or io['DI0']!=0:
                raise RuntimeError('Empty-arm return requires current non-holding feedback')
        if a.initial_holding:wait_io(1,io_seq);holding=True
        operations=recipe['operations']
        if a.place_only:
            if not a.initial_holding:raise ValueError('place-only requires initial-holding')
            start=next(i for i,op in enumerate(operations) if op['type']=='bind_async_result')
            operations=operations[start:]
        for op in operations:
            kind=op['type'];record('operation_start',operation=op)
            if kind=='servoj':
                ref=json.loads((a.cycle.parent/op['reference']).read_text())
                from run_servoj_executor import preserve_reference_braking,subdivide_reference
                original_count=len(ref['positions_deg'])
                ref['positions_deg']=preserve_reference_braking(
                    ref['positions_deg'],ref['period_s'],
                    min(profile['reference_acceleration_cap_deg_s2']))
                record('reference_acceleration_timing',source_points=original_count,
                    sent_points=len(ref['positions_deg']),
                    acceleration_cap_deg_s2=min(profile['reference_acceleration_cap_deg_s2']),
                    sent_duration_s=(len(ref['positions_deg'])-1)*ref['period_s'])
                velocity_rows=ref.get('velocities_deg_s')
                if velocity_rows is not None:
                    factor=(len(ref['positions_deg'])-1)//(original_count-1) if original_count>1 else 1
                    velocity_rows=[[v/factor for v in row] for row in subdivide_reference(velocity_rows,factor)]
                goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=joint_names
                for i,q in enumerate(ref['positions_deg']):
                    ns=round(i*ref['period_s']*1e9)
                    goal.trajectory.points.append(JointTrajectoryPoint(
                        positions=[math.radians(v) for v in q],
                        velocities=[] if velocity_rows is None else [math.radians(v) for v in velocity_rows[i]],
                        time_from_start=Duration(sec=ns//1000000000,nanosec=ns%1000000000)))
                report['motion_requested']=True
                active=await_future(action.send_goal_async(goal))
                if not active.accepted:raise RuntimeError('Executor rejected trajectory')
                future=active.get_result_async()
                while not future.done() and rclpy.ok():
                    rclpy.spin_once(node,timeout_sec=.02)
                    if holding and io is not None and io['DI0']!=1:
                        lost_io=dict(io)
                        record('holding_feedback_lost',value=lost_io)
                        await_future(active.cancel_goal_async())
                        raise RuntimeError(f'Holding feedback lost (DI0={lost_io["DI0"]}, DO0={lost_io["DO0"]}, '
                            f'DO1={lost_io["DO1"]}); trajectory canceled, vacuum retained')
                result=await_future(future);active=None
                record('trajectory_result',status=result.status,error_code=result.result.error_code,
                    error_string=result.result.error_string,actual_joint_sample=latest_q)
                if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code!=0:
                    raise RuntimeError('Trajectory did not succeed')
            elif kind=='vacuum_on_and_confirm':
                valve(1,False);valve(0,True);wait_io(1,io_seq);holding=True
            elif kind=='bind_async_result':
                # References are already bound to one observation and prefix by
                # compose_cycle. The real executor checks each new motion goal.
                record('entry_feedback',scene_version=op['scene_version'],actual_joint_sample=latest_q)
            elif kind in ('release_and_confirm_support','release_and_confirm_vacuum_off'):
                report['release_joint_sample']=latest_q
                holding=False;valve(0,False);valve(1,True);wait_io(0,io_seq);valve(1,False)
                record('release_di_confirmed',support_observed=False)
            elif kind=='commit_planned_occupancy':
                # Explicit task assumption, not a detector or settling result.
                state=recipe['next_scene_after_execution']
                (a.output/'scene_state.json').write_text(json.dumps(state,indent=2))
                report.update(occupancy_committed=True,
                    occupancy_source='planned_release_pose_assumption',
                    next_scene=str(a.output/'scene_state.json'),
                    status='motion_and_io_finished_with_planned_occupancy')
            else:raise ValueError('Unknown operation '+kind)
            record('operation_end',type=kind)
        report['motion_and_io_sequence_finished']=True
        report['completed']=True
    except BaseException as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
        record('stopped',error=report['error'])
        if active is not None:
            try:await_future(active.cancel_goal_async())
            except Exception as cancel_error:record('cancel_error',message=str(cancel_error))
        # A failed loaded motion must not automatically drop the payload.
        raise
    finally:
        (a.output/'result.json').write_text(json.dumps(report,indent=2))
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
