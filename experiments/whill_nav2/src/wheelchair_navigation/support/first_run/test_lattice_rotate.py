#!/usr/bin/env python3
"""Planner-only rotation test in an isolated ROS domain, no hardware topics."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import Transition
from nav2_msgs.action import ComputePathToPose

ROOT = Path('/home/whill/Shaki_ws/src/WheelChair_ws')
sys.path.insert(0, str(ROOT / 'tools'))
from test_hybrid_planner import TestNode, spin_for, transition, wait_future

if os.environ.get('ROS_DOMAIN_ID') != '198' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('requires ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1')
if Path(get_package_prefix('nav2_planner')) != ROOT / 'install/nav2_planner':
    raise RuntimeError('wrong planner installation')
rclpy.init()
node = TestNode()
planner = None
try:
    spin_for(node, 2)
    other_nodes = [name for name in node.get_node_names() if name != node.get_name()]
    if other_nodes:
        raise RuntimeError(f'test domain in use: {other_nodes}')
    node.begin_synthetic_data()
    planner = subprocess.Popen([str(ROOT / 'install/nav2_planner/lib/nav2_planner/planner_server'),
                                '--ros-args', '--params-file', str(ROOT / 'first_run/lattice_test.yaml')])
    transition(node, Transition.TRANSITION_CONFIGURE)
    transition(node, Transition.TRANSITION_ACTIVATE)
    spin_for(node, 3)
    if not node.action.wait_for_server(timeout_sec=10):
        raise RuntimeError('planner action unavailable')
    goal = ComputePathToPose.Goal()
    goal.planner_id = 'GridBased'
    goal.use_start = True
    for pose in (goal.start, goal.goal):
        pose.header.frame_id = 'odom'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = -2.0
    goal.start.pose.orientation.w = 1.0
    goal.goal.pose.orientation.z = math.sin(math.pi / 4)
    goal.goal.pose.orientation.w = math.cos(math.pi / 4)
    handle = wait_future(node, node.action.send_goal_async(goal), 15)
    if not handle.accepted:
        raise RuntimeError('goal rejected')
    response = wait_future(node, handle.get_result_async(), 30)
    poses = response.result.path.poses
    if response.status != GoalStatus.STATUS_SUCCEEDED or len(poses) < 2:
        raise RuntimeError(f'failed: status={response.status} poses={len(poses)}')
    displacement = max(math.hypot(p.pose.position.x + 2.0, p.pose.position.y) for p in poses)
    q = poses[-1].pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    error = abs(math.atan2(math.sin(yaw-math.pi/2), math.cos(yaw-math.pi/2)))
    print(f'rotation path: poses={len(poses)}, max translation={displacement:.4f}m, '
          f'final yaw={math.degrees(yaw):.1f}deg, yaw error={math.degrees(error):.1f}deg', flush=True)
    if displacement > 0.08 or error > math.radians(10):
        raise RuntimeError('rotation path exceeds test tolerances')
finally:
    if planner is not None and planner.poll() is None:
        planner.send_signal(signal.SIGINT)
        try:
            planner.wait(timeout=10)
        except subprocess.TimeoutExpired:
            planner.terminate()
            planner.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()
