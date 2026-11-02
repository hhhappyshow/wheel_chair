#!/usr/bin/env python3
import copy
import json
from pathlib import Path
import yaml
here=Path(__file__).resolve().parent
data=yaml.safe_load((here/'navigation.yaml').read_text())
before=copy.deepcopy(data)
data['controller_server']['ros__parameters']['FollowPath']['max_angular_accel']=0.8
data['behavior_server']['ros__parameters']['rotational_acc_lim']=0.8
data['velocity_smoother']['ros__parameters']['max_accel'][2]=0.8
data['velocity_smoother']['ros__parameters']['max_decel'][2]=-0.8
with (here/'navigation_angular_accel_08.yaml').open('x') as f:yaml.safe_dump(data,f,sort_keys=False)
sim=copy.deepcopy(data)
def convert(obj):
    if isinstance(obj,dict):
        for k,v in obj.items():
            if k=='use_sim_time':obj[k]=True
            else:convert(v)
    elif isinstance(obj,list):
        for v in obj:convert(v)
convert(sim)
sim['collision_monitor']['ros__parameters']['cmd_vel_out_topic']='/wheelchair/sim/drive'
with (here/'simulation_angular_accel_08.yaml').open('x') as f:yaml.safe_dump(sim,f,sort_keys=False)
diff=[]
def compare(a,b,path=''):
    if isinstance(a,dict):
        assert a.keys()==b.keys()
        for k in a:compare(a[k],b[k],path+'.'+k)
    elif a!=b:diff.append({'parameter':path,'before':a,'after':b})
compare(before,data)
assert len(diff)==4
with (here/'angular_accel_08_diff.json').open('x') as f:json.dump(diff,f,indent=2)
print(json.dumps(diff))
