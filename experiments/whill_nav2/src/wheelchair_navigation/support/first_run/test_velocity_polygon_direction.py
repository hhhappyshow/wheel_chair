#!/usr/bin/env python3
"""Exercise the isolated backport; never publish to production topics."""
import math
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import yaml

if os.environ.get('ROS_DOMAIN_ID') != '198' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('requires isolated ROS_DOMAIN_ID=198 and ROS_LOCALHOST_ONLY=1')

root = '/home/whill/Shaki_ws/src/WheelChair_ws'
with open(root + '/first_run/nav2_bidirectional_full.yaml', encoding='utf-8') as f:
    params = yaml.safe_load(f)['collision_monitor']['ros__parameters']
params['polygons'] = ['StopZone']
params['cmd_vel_in_topic'] = '/test/input'
params['cmd_vel_out_topic'] = '/test/output'
params['scan']['topic'] = '/test/scan'
params['base_shift_correction'] = False
params['source_timeout'] = 0.75
params['StopZone'] = {
    'type': 'velocity_polygon', 'action_type': 'stop', 'max_points': 2,
    'enabled': True, 'visualize': False, 'holonomic': False,
    'velocity_polygons': ['reverse_straight', 'forward_straight', 'fallback'],
    'reverse_straight': {'points': '[[-0.55,-0.33],[0.82,-0.33],[0.82,0.33],[-0.55,0.33]]',
                         'linear_min': -2.0, 'linear_max': -0.001,
                         'theta_min': -0.01, 'theta_max': 0.01},
    'forward_straight': {'points': '[[-0.23,-0.38],[1.05,-0.38],[1.05,0.38],[-0.23,0.38]]',
                         'linear_min': 0.001, 'linear_max': 2.0,
                         'theta_min': -0.01, 'theta_max': 0.01},
    'fallback': {'points': '[[-0.55,-0.38],[1.05,-0.38],[1.05,0.38],[-0.55,0.38]]',
                 'linear_min': -10.0, 'linear_max': 10.0,
                 'theta_min': -10.0, 'theta_max': 10.0},
}

rclpy.init()
node = Node('stopzone_direction_test')
if [name for name in node.get_node_names() if name != node.get_name()]:
    raise RuntimeError('test domain is already in use')
proc = None
try:
    with tempfile.TemporaryDirectory(prefix='stopzone-test-') as temp:
        config = temp + '/monitor.yaml'
        with open(config, 'w', encoding='utf-8') as f:
            yaml.safe_dump({'collision_monitor': {'ros__parameters': params}}, f)
        proc = subprocess.Popen([
            root + '/velocity_polygon_overlay_20261031/install/nav2_collision_monitor/lib/nav2_collision_monitor/collision_monitor',
            '--ros-args', '--params-file', config,
        ])
        client = node.create_client(ChangeState, '/collision_monitor/change_state')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('lifecycle service unavailable')
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            req = ChangeState.Request()
            req.transition.id = transition
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done() or not future.result().success:
                raise RuntimeError(f'lifecycle transition {transition} failed')

        scan_pub = node.create_publisher(LaserScan, '/test/scan', qos_profile_sensor_data)
        cmd_pub = node.create_publisher(Twist, '/test/input', 10)
        outputs = []
        node.create_subscription(Twist, '/test/output', outputs.append, 10)

        def exercise(v, w, obstacle):
            outputs.clear()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                scan = LaserScan()
                scan.header.frame_id = 'base_link'
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.angle_min = 0.37
                scan.angle_increment = 0.01
                scan.angle_max = 0.41
                scan.range_min = 0.01
                scan.range_max = 10.0
                scan.ranges = [0.92] * 5 if obstacle else [math.inf] * 5
                scan_pub.publish(scan)
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                cmd_pub.publish(cmd)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.04)
            if not outputs:
                raise RuntimeError('no collision monitor output')
            out = outputs[-1]
            return out.linear.x, out.angular.z

        for label, v, w, should_stop in [('forward', 0.1, 0.0, True),
                                        ('reverse', -0.1, 0.0, False),
                                        ('rotate_right', 0.0, -0.2, True)]:
            clear = exercise(v, w, False)
            blocked = exercise(v, w, True)
            print(f'{label}: clear={clear}, left_front_obstacle={blocked}', flush=True)
            if abs(clear[0] - v) > 1e-6 or abs(clear[1] - w) > 1e-6:
                raise RuntimeError('clear input did not pass through')
            expected = (0.0, 0.0) if should_stop else (v, w)
            if abs(blocked[0] - expected[0]) > 1e-6 or abs(blocked[1] - expected[1]) > 1e-6:
                raise RuntimeError('unexpected velocity polygon result')

        # Stop publishing scans while continuing commands: test stale-source behavior.
        exercise(0.1, 0.0, False)
        outputs.clear()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            cmd = Twist()
            cmd.linear.x = 0.1
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.04)
        if not outputs:
            raise RuntimeError('no output after stopping scan')
        print(f'stale_scan: output=({outputs[-1].linear.x}, {outputs[-1].angular.z})', flush=True)
        if abs(outputs[-1].linear.x) > 1e-6 or abs(outputs[-1].angular.z) > 1e-6:
            raise RuntimeError('stale scan failed open')
finally:
    if proc is not None and proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.terminate()
            proc.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()
