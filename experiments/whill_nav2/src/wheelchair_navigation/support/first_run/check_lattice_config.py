#!/usr/bin/env python3
"""Check that only the planner and progress checker changed."""
import sys
import yaml

with open(sys.argv[1]) as stream:
    old = yaml.safe_load(stream)
with open(sys.argv[2]) as stream:
    new = yaml.safe_load(stream)
planner = new['planner_server']['ros__parameters']['GridBased']
progress = new['controller_server']['ros__parameters']['progress_checker']
assert planner['plugin'] == 'nav2_smac_planner/SmacPlannerLattice'
assert planner['allow_reverse_expansion'] is False
assert planner['allow_unknown'] is False
assert progress['plugin'] == 'nav2_controller::PoseProgressChecker'
assert progress['required_movement_angle'] == 0.10
old['planner_server']['ros__parameters']['GridBased'] = planner
old['controller_server']['ros__parameters']['progress_checker'] = progress
if old != new:
    raise SystemExit('Unexpected changes beyond planner and progress checker')
print('Only planner and progress checker changed; limits and collision config preserved.')
