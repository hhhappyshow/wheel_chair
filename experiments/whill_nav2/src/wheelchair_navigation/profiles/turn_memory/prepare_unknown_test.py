#!/usr/bin/env python3
from pathlib import Path
import yaml

here = Path(__file__).resolve().parent
data = yaml.safe_load((here/'simulation.yaml').read_text())
for name in ['global_costmap','local_costmap']:
    p = data[name][name]['ros__parameters']
    p['static_layer']['map_topic'] = '/turn_memory_test/unknown_map'
    p['memory_obstacles']['scan']['topic'] = '/turn_memory_test/no_returns'
with (here/'simulation_unknown.yaml').open('x') as f:
    yaml.safe_dump(data,f,sort_keys=False)
