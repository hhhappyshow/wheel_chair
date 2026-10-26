"""Read current S20 feedback through an already authenticated SSH socket."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess

PROGRAM = r'''
import json, math, sys, time
from datetime import datetime, timezone
sys.path.insert(0, '/home/hku_reconova/RoboticArm-master/third_party/huayan_python_sdk')
import CPS
c=CPS.CPSClient()
c.g_clients=[CPS.RbtClient() for _ in range(c.MaxBox)]
c.g_plugin_clients=[CPS.PluginClient() for _ in range(c.MaxBox)]
c.g_client_state=[False]*c.MaxBox
c.g_plugin_client_state=[False]*c.MaxBox
def read(name):
    values=[]
    code=getattr(c,name)(0,0,values)
    if code:raise RuntimeError('%s returned %s'%(name,code))
    return values
connected=False
try:
    code=c.HRIF_Connect(0,'192.168.0.10',10003)
    if code:raise RuntimeError('HRIF_Connect returned %s'%code)
    connected=True
    samples=[]
    for _ in range(3):
        q=list(map(float,read('HRIF_ReadActJointPos')[:6]))
        samples.append(dict(stamp_utc=datetime.now(timezone.utc).isoformat(),q_deg=q,
                            q_rad=[math.radians(v) for v in q],
                            act_pos_raw=read('HRIF_ReadActPos'),
                            qd_deg_s=list(map(float,read('HRIF_ReadActJointVel')[:6]))))
        time.sleep(.2)
    result=dict(samples=samples,q_deg=samples[-1]['q_deg'],q_rad=samples[-1]['q_rad'],
                tcp_config_raw=read('HRIF_ReadCurTCP'),ucs_config_raw=read('HRIF_ReadCurUCS'),
                fsm_raw=read('HRIF_ReadCurFSM'),motion_sent=False,io_written=False,
                controller_configuration_written=False,source='fresh vendor SDK read-only feedback')
    print('S20_POSE_JSON='+json.dumps(result),flush=True)
finally:
    if connected:c.HRIF_DisConnect(0)
    heartbeat=getattr(c.g_clients[0],'hb_thread',None)
    if heartbeat is not None:heartbeat.join(timeout=2.)
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    proc=subprocess.run(['ssh','-S','/tmp/codex-s20-readonly-20261024',
        'hku_reconova@192.168.3.12','python3','-u','-'],input=PROGRAM,text=True,
        capture_output=True,timeout=30)
    lines=[s for s in proc.stdout.splitlines() if s.startswith('S20_POSE_JSON=')]
    if proc.returncode or not lines:raise RuntimeError(proc.stderr+proc.stdout)
    result=json.loads(lines[-1].split('=',1)[1])
    result['captured_at_client_local']=datetime.now().astimezone().isoformat()
    result['remote_clock_warning']='Jetson clock may be uninitialized; remote sample timestamps are not trusted wall-clock time'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
