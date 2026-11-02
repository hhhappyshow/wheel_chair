#!/usr/bin/env python3
"""Create a separate, low-speed Nav2 configuration from the existing baseline."""
from copy import deepcopy
from pathlib import Path
import yaml

root = Path('/home/whill/Shaki_ws/src/WheelChair_ws/first_run')
source = Path('/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/wheelchair_nav2_pipeline/config/navigation.yaml')
with source.open() as stream:
    params = deepcopy(yaml.safe_load(stream))

def replace_frames(obj):
    if isinstance(obj, dict):
        return {k: replace_frames(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [replace_frames(v) for v in obj]
    return {'wheelchair_control_center_estimated': 'base_link',
            '/wheelchair/control_center/odom': '/wheelchair/odom'}.get(obj, obj) if isinstance(obj, str) else obj

params = replace_frames(params)
params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(root / 'navigate_no_recovery.xml')
params['planner_server']['ros__parameters']['GridBased'] = {
    'plugin': 'nav2_smac_planner/SmacPlannerLattice',
    'lattice_filepath': '/home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/wheelchair_nav2/config/wheelchair_lattice.json',
    'allow_unknown': False,
    'allow_reverse_expansion': False,
    'smooth_path': False,
    'tolerance': 0.05,
    'rotation_penalty': 1.0,
    'analytic_expansion_max_length': 0.0,
}
ctrl = params['controller_server']['ros__parameters']
ctrl['progress_checker']['plugin'] = 'nav2_controller::PoseProgressChecker'
ctrl['progress_checker']['required_movement_angle'] = 0.10
ctrl['progress_checker']['required_movement_radius'] = 0.05
ctrl['progress_checker']['movement_time_allowance'] = 15.0
ctrl['general_goal_checker']['xy_goal_tolerance'] = 0.10
ctrl['general_goal_checker']['yaw_goal_tolerance'] = 0.17453292519943295
ctrl['general_goal_checker']['stateful'] = False
ctrl['FollowPath'].update({'vx_max': 0.536622219601921, 'vx_min': 0.0,
                           'wz_max': 0.8108854254075847})
for name in ('local_costmap', 'global_costmap'):
    costmap = params[name][name]['ros__parameters']
    costmap['inflation_layer']['inflation_radius'] = 0.30
    costmap.pop('filters', None)
    costmap.pop('keepout_filter', None)
smooth = params['velocity_smoother']['ros__parameters']
smooth.update({'max_velocity': [0.536622219601921, 0.0, 0.8108854254075847],
               'min_velocity': [0.0, 0.0, -0.8108854254075847],
               'max_accel': [0.30, 0.0, 0.50],
               'max_decel': [-0.30, 0.0, -0.50],
               'velocity_timeout': 0.20})
monitor = params['collision_monitor']['ros__parameters']
monitor['source_timeout'] = 0.75
monitor['polygons'] = ['StopZone', 'FootprintApproach']
monitor['StopZone'] = {'type': 'polygon', 'action_type': 'stop',
                       'points': [1.05, 0.38, 1.05, -0.38, -0.30, -0.38, -0.30, 0.38],
                       'max_points': 2, 'visualize': False, 'enabled': True}
monitor['FootprintApproach']['max_points'] = 2
out = root / 'nav2_low_speed.yaml'
with out.open('x') as stream:
    yaml.safe_dump(params, stream, sort_keys=False)
print(out)
