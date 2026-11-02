#!/usr/bin/env python3
"""Align the existing official Spin server's limits within the third profile only."""
import json
from pathlib import Path
import yaml
here=Path(__file__).resolve().parent
changes={}
for name in ['navigation.yaml','simulation.yaml','simulation_unknown.yaml']:
    p=here/name
    data=yaml.safe_load(p.read_text())
    b=data['behavior_server']['ros__parameters']
    changes[name]={k:b[k] for k in ['max_rotational_vel','min_rotational_vel','rotational_acc_lim']}
    b.update(max_rotational_vel=0.8108854254075847,min_rotational_vel=0.1,rotational_acc_lim=0.6)
    backup=p.with_suffix('.before_spin_alignment.yaml')
    with backup.open('x') as f:f.write(p.read_text())
    p.write_text(yaml.safe_dump(data,sort_keys=False))
with (here/'spin_limit_alignment.json').open('x') as f:json.dump(changes,f,indent=2)
