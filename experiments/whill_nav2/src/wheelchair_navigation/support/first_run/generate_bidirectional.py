#!/usr/bin/env python3
"""Create a separate forward/reverse config without modifying the running config."""
from copy import deepcopy
import json
from pathlib import Path
import yaml

root = Path('/home/whill/Shaki_ws/src/WheelChair_ws/first_run')
model = json.loads(Path('/home/whill/Shaki_ws/esp32_velocity_interface/calibration/model.json').read_text())
forward_max = model['input_radius_steps'] * model['forward_gain']
reverse_max = model['input_radius_steps'] * model['reverse_gain']
angular_max = model['input_radius_steps'] * model['turn_gain']
with (root / 'nav2_low_speed.yaml').open() as stream:
    old = yaml.safe_load(stream)
params = deepcopy(old)
grid = params['planner_server']['ros__parameters']['GridBased']
if grid.get('plugin') != 'nav2_smac_planner/SmacPlannerLattice':
    raise SystemExit('Expected the verified Lattice planner')
grid['allow_reverse_expansion'] = True
ctrl = params['controller_server']['ros__parameters']['FollowPath']
if ctrl.get('plugin') != 'nav2_mppi_controller::MPPIController':
    raise SystemExit('Expected MPPI controller')
ctrl['vx_max'] = forward_max
ctrl['vx_min'] = -reverse_max
ctrl['wz_max'] = angular_max
ctrl['enforce_path_inversion'] = True
ctrl['inversion_xy_tolerance'] = 0.10
ctrl['inversion_yaw_tolerance'] = 0.17453292519943295
ctrl.setdefault('PathAngleCritic', {})['forward_preference'] = False
ctrl.setdefault('PreferForwardCritic', {})['enabled'] = False
smooth = params['velocity_smoother']['ros__parameters']
smooth['max_velocity'][0] = forward_max
smooth['max_velocity'][2] = angular_max
smooth['min_velocity'][0] = -reverse_max
smooth['min_velocity'][2] = -angular_max
monitor = params['collision_monitor']['ros__parameters']
monitor['StopZone']['points'] = [0.8, 0.3, 0.8, -0.3, -0.15, -0.3, -0.15, 0.3]
monitor['StopZone']['max_points'] = 5
output = root / 'nav2_bidirectional_full.yaml'
with output.open('x') as stream:
    yaml.safe_dump(params, stream, sort_keys=False)
print(output)
