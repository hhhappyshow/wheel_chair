#!/usr/bin/env python3
"""Evaluate the upstream braking patch with command-based angular acceleration."""
from pathlib import Path
import copy
import yaml
here=Path(__file__).resolve().parent
hardware=yaml.safe_load((here/'navigation.yaml').read_text())
assert hardware['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
hardware['controller_server']['ros__parameters']['FollowPath']['closed_loop']=False
with (here/'navigation_rotation_smooth.yaml').open('x') as f:yaml.safe_dump(hardware,f,sort_keys=False)
sim=copy.deepcopy(hardware)
def convert(obj):
    if isinstance(obj,dict):
        for k,v in obj.items():
            if k=='use_sim_time':obj[k]=True
            else:convert(v)
    elif isinstance(obj,list):
        for v in obj:convert(v)
convert(sim)
sim['collision_monitor']['ros__parameters']['cmd_vel_out_topic']='/wheelchair/sim/drive'
with (here/'simulation_rotation_smooth.yaml').open('x') as f:yaml.safe_dump(sim,f,sort_keys=False)
