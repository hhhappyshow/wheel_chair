#!/usr/bin/env python3
"""Require fresh dual MID-360 and Odin odometry before enabling Nav2."""
import time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = rclpy.create_node('shaki_sensor_startup_check')
messages = {}
for name in ('/mid360_201/points_filtered', '/mid360_202/points_filtered'):
    node.create_subscription(PointCloud2, name, lambda m, key=name: messages.__setitem__(key, m), qos_profile_sensor_data)
node.create_subscription(Odometry, '/wheelchair/odom', lambda m: messages.__setitem__('odom', m), qos_profile_sensor_data)
deadline = time.monotonic() + 35
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if len(messages) != 3:
        continue
    now = node.get_clock().now().nanoseconds / 1e9
    ages = {key: now-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9 for key, msg in messages.items()}
    if all(-0.05 <= age <= (0.3 if key == 'odom' else 0.75) for key, age in ages.items()):
        odom = messages['odom']
        if odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_link':
            print('Sensors ready:', ', '.join(f'{key}={age:.3f}s' for key, age in ages.items()), flush=True)
            node.destroy_node()
            rclpy.shutdown()
            raise SystemExit(0)
print(f'Sensors missing, stale, or wrong odometry frames: {list(messages)}', flush=True)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
