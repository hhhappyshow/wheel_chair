#!/usr/bin/env python3
"""Verify a backward path in a separate domain with no hardware controller."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys

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
    others = [name for name in node.get_node_names() if name != node.get_name()]
    if others:
        raise RuntimeError(f'test domain in use: {others}')
    node.begin_synthetic_data()
    planner = subprocess.Popen([str(ROOT / 'install/nav2_planner/lib/nav2_planner/planner_server'),
                                '--ros-args', '--params-file', str(ROOT / 'first_run/lattice_reverse_test.yaml')])
    transition(node, Transition.TRANSITION_CONFIGURE)
    transition(node, Transition.TRANSITION_ACTIVATE)
    spin_for(node, 3)
    if not node.action.wait_for_server(timeout_sec=10):
        raise RuntimeError('planner action unavailable')
    goal = ComputePathToPose.Goal()
    goal.planner_id, goal.use_start = 'GridBased', True
    for pose in (goal.start, goal.goal):
        pose.header.frame_id = 'odom'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
    goal.start.pose.position.x, goal.goal.pose.position.x = -2.0, -2.5
    handle = wait_future(node, node.action.send_goal_async(goal), 15)
    if not handle.accepted:
        raise RuntimeError('goal rejected')
    response = wait_future(node, handle.get_result_async(), 30)
    poses = response.result.path.poses
    if response.status != GoalStatus.STATUS_SUCCEEDED or len(poses) < 2:
        raise RuntimeError(f'failed status={response.status}, poses={len(poses)}')
    backwards = 0.0
    for a, b in zip(poses, poses[1:]):
        q = a.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        dx = b.pose.position.x-a.pose.position.x
        dy = b.pose.position.y-a.pose.position.y
        along = dx*math.cos(yaw)+dy*math.sin(yaw)
        backwards += max(0.0, -along)
    end = poses[-1].pose.position
    error = math.hypot(end.x+2.5, end.y)
    print(f'backward path: poses={len(poses)}, backward distance={backwards:.3f}m, endpoint error={error:.3f}m', flush=True)
    if backwards < 0.30 or error > 0.05:
        raise RuntimeError('no sufficient backward motion in path')
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
