#!/usr/bin/env python3
"""Verify that regeneration changes only the requested speed and acceleration limits."""
import sys
import yaml

with open(sys.argv[1]) as stream:
    old = yaml.safe_load(stream)
with open(sys.argv[2]) as stream:
    new = yaml.safe_load(stream)
expected_ctrl = old['controller_server']['ros__parameters']['FollowPath']
expected_ctrl.update({'vx_max': 0.536622219601921, 'vx_min': 0.0,
                      'wz_max': 0.8108854254075847})
expected_smoother = old['velocity_smoother']['ros__parameters']
expected_smoother.update({'max_velocity': [0.536622219601921, 0.0, 0.8108854254075847],
                          'min_velocity': [0.0, 0.0, -0.8108854254075847],
                          'max_accel': [0.30, 0.0, 0.50],
                          'max_decel': [-0.30, 0.0, -0.50]})
if old != new:
    raise SystemExit('Unexpected config differences; refusing restart')
print('Only requested velocity and acceleration limits changed.')
