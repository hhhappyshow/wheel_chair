#!/usr/bin/env python3
"""Copy the existing RViz layout and match this Nav2 graph's path QoS."""
from pathlib import Path
import yaml

source = Path('/home/whill/eye_ctrl_wheelchair/artifacts/native_config_withdrawn_20261030/pure_nav2.rviz')
target = Path('/home/whill/Shaki_ws/src/WheelChair_ws/first_run/nav2_goal.rviz')
with source.open() as stream:
    config = yaml.safe_load(stream)
manager = config['Visualization Manager']
manager['Global Options']['Fixed Frame'] = 'odom'
for display in manager['Displays']:
    if display.get('Class') != 'rviz_common/Group':
        continue
    display['Displays'] = [d for d in display.get('Displays', [])
                           if d.get('Topic', {}).get('Value') != '/wheelchair/debug/executed_path']
    for item in display['Displays']:
        if item.get('Topic', {}).get('Value') == '/plan':
            item['Name'] = 'Nav2 planned path'
            item['Topic']['Durability Policy'] = 'Volatile'
manager['Displays'] = [d for d in manager['Displays']
                       if d.get('Topic', {}).get('Value') != '/wheelchair/display/stream_status']
with target.open('x') as stream:
    yaml.safe_dump(config, stream, sort_keys=False)
print(target)
