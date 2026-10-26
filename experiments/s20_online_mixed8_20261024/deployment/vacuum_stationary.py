"""Explicit supported-box valve diagnostic, without any robot motion goal."""
import argparse
import json
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--supported',action='store_true')
    a=p.parse_args()
    if not (a.execute and a.supported):
        raise ValueError('Requires explicit execution and operator-confirmed box support')
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Node('s20_stationary_valve_diagnostic')
    rows=[];phase='initial';report=dict(motion_sent=False,io_requested=False,completed=False,stages=[])
    log=(a.output/'events.jsonl').open('x')
    def record(event,**values):
        row=dict(event=event,phase=phase,monotonic_s=time.monotonic(),**values)
        log.write(json.dumps(row)+'\n');log.flush()
        return row
    def receive(msg):
        data=json.loads(msg.data)
        if data.get('schema')=='elfin_vacuum_io/v1':rows.append(record('io',value=data))
    node.create_subscription(String,'/vacuum/io',receive,10)
    def valve(bit,on):
        client=node.create_client(SetBool,f'/elfin/vacuum/set_do{bit}')
        try:
            if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('Valve service unavailable')
            report['io_requested']=True
            future=client.call_async(SetBool.Request(data=on))
            rclpy.spin_until_future_complete(node,future,timeout_sec=3.)
            if not future.done():raise RuntimeError('Valve service response missing')
            result=future.result()
            record('valve',bit=bit,on=on,success=result.success,message=result.message)
            if not result.success:raise RuntimeError(result.message)
        finally:node.destroy_client(client)
    def sample(seconds):
        began=time.monotonic();start=len(rows)
        while time.monotonic()-began<seconds:rclpy.spin_once(node,timeout_sec=.02)
        values=[v['value'] for v in rows[start:]]
        transitions=sum(x['DI0']!=y['DI0'] for x,y in zip(values,values[1:]))
        report['stages'].append(dict(phase=phase,seconds=time.monotonic()-began,
            messages=len(values),DI0_one=sum(v['DI0']==1 for v in values),
            DI0_zero=sum(v['DI0']==0 for v in values),DI0_transitions=transitions,
            final_io=values[-1] if values else None))
    try:
        sample(.5)
        phase='release_before';valve(0,False);valve(1,True);sample(2.);valve(1,False)
        phase='suction';valve(1,False);valve(0,True);sample(3.)
        phase='release_after';valve(0,False);valve(1,True);sample(2.);valve(1,False)
        report['completed']=True
    except BaseException as exc:
        report['error']=str(exc)
        raise
    finally:
        (a.output/'result.json').write_text(json.dumps(report,indent=2))
        log.close();node.destroy_node();rclpy.shutdown()
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
