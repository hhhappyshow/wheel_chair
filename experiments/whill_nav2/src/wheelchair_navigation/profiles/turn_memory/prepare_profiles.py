#!/usr/bin/env python3
"""Independent configuration using existing Nav2 plugins; retain source profiles."""
import copy
import hashlib
import json
from pathlib import Path
import yaml

here = Path(__file__).resolve().parent
root = Path('/home/whill/Shaki_ws/src/WheelChair_ws/runtime/nav2_humble_update_20261031')
source = root/'rotation_shim_20261101/hardware_lattice_shim.yaml'
data = yaml.safe_load(source.read_text())
data['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
data['controller_server']['ros__parameters']['FollowPath']['closed_loop'] = False
data['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'] = 0.3
data['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'] = 0.3490658503988659
data['controller_server']['ros__parameters']['FollowPath']['max_angular_accel'] = 0.8
data['velocity_smoother']['ros__parameters']['max_accel'][2] = 0.8
data['velocity_smoother']['ros__parameters']['max_decel'][2] = -0.8
data['behavior_server']['ros__parameters'].update(
    max_rotational_vel=0.8108854254075847, min_rotational_vel=0.1, rotational_acc_lim=0.8)
for zone in ['forward_straight', 'reverse_straight', 'fallback']:
    data['collision_monitor']['ros__parameters']['StopZone'][zone]['points'] = '[[-0.15,-0.28],[0.77,-0.28],[0.77,0.28],[-0.15,0.28]]'
for name in ['global_costmap', 'local_costmap']:
    p = data[name][name]['ros__parameters']
    p['footprint'] = '[[-0.18,-0.28],[0.77,-0.28],[0.77,0.28],[-0.18,0.28]]'
    p['plugins'] = ['static_layer', 'memory_obstacles', 'inflation_layer']
    p['memory_obstacles'] = {
        'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
        'footprint_clearing_enabled': False, 'combination_method': 1,
        'min_obstacle_height': -1.0, 'max_obstacle_height': 1.0,
        'observation_sources': 'scan',
        'scan': {'topic': '/wheelchair/memory_scan', 'data_type': 'LaserScan',
                 'marking': True, 'clearing': True, 'inf_is_valid': False,
                 'observation_persistence': 0.0, 'expected_update_rate': 0.0,
                 'min_obstacle_height': -1.0, 'max_obstacle_height': 1.0,
                 'obstacle_min_range': 0.0, 'obstacle_max_range': 8.0,
                 'raytrace_min_range': 0.0, 'raytrace_max_range': 8.5}}
data['local_costmap']['local_costmap']['ros__parameters']['track_unknown_space'] = False
hardware = here/'navigation.yaml'
with hardware.open('x') as f:
    yaml.safe_dump(data, f, sort_keys=False)
sim = copy.deepcopy(data)
def convert(obj):
    if isinstance(obj, dict):
        for k,v in obj.items():
            if k == 'use_sim_time': obj[k] = True
            else: convert(v)
    elif isinstance(obj, list):
        for v in obj: convert(v)
convert(sim)
sim['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] = '/wheelchair/sim/drive'
with (here/'simulation.yaml').open('x') as f:
    yaml.safe_dump(sim, f, sort_keys=False)
protected = [root/'hardware_lattice_10hz.yaml', root/'start_navigation_lattice_10hz.sh',
             root/'hybrid025_20261101/hardware_hybrid025_10hz_analytic125.yaml',
             root/'hybrid025_20261101/start_navigation_hybrid025_10hz.sh']
receipt = {'base': str(source), 'base_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
           'protected_sha256': {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in protected},
           'new_profile': str(hardware), 'changes': ['planner allow_unknown=false (latest user request)',
           'local track_unknown_space=false', 'official memory_obstacles added to both maps',
           'native Spin angular limits aligned with existing executable limits'],
           'filter_mask': {'source': 'three-board clearance revision 20261101', 'box': [-.38,.77,-.28,.28]},
           'collision_monitor_input_unchanged': '/wheelchair/scan'}
with (here/'receipt.json').open('x') as f:
    json.dump(receipt, f, indent=2)
print(json.dumps(receipt, indent=2))
