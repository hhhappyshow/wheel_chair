#!/usr/bin/env python3
"""Read-only, timestamped trace of navigation commands and actual odometry."""
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    output = Path(sys.argv[1])
    if output.exists():
        raise SystemExit('Trace exists')
    rclpy.init()
    node = rclpy.create_node('shaki_live_readonly_trace')
    data = {key: [] for key in ('raw', 'smooth', 'output', 'odom', 'plan', 'goal', 'status', 'interface', 'scan', 'mapping')}
    now = lambda: node.get_clock().now().nanoseconds/1e9
    last = {'odom': 0., 'interface': 0.}
    def record_twist(msg, key):
        data[key].append({'t': now(), 'v': msg.linear.x, 'w': msg.angular.z})
    for key, topic in [('raw', '/wheelchair/nav2/cmd_vel_raw'),
                       ('smooth', '/wheelchair/nav2/cmd_vel_smoothed'),
                       ('output', '/wheelchair/nav2/cmd_vel')]:
        node.create_subscription(Twist, topic, lambda msg, key=key: record_twist(msg, key), 50)
    def odom(msg):
        t = now()
        if t-last['odom'] < .015:
            return
        last['odom'] = t
        p = msg.pose.pose
        data['odom'].append({'t': t, 'stamp': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                             'x': p.position.x, 'y': p.position.y, 'yaw': angle(p.orientation),
                             'v': msg.twist.twist.linear.x, 'w': msg.twist.twist.angular.z})
    node.create_subscription(Odometry, '/wheelchair/odom', odom, qos_profile_sensor_data)
    def plan(msg):
        data['plan'].append({'t': now(), 'frame': msg.header.frame_id,
                             'poses': [[p.pose.position.x, p.pose.position.y, angle(p.pose.orientation)] for p in msg.poses]})
    node.create_subscription(NavPath, '/plan', plan, 30)
    node.create_subscription(PoseStamped, '/goal_pose',
                             lambda p: data['goal'].append({'t': now(), 'frame': p.header.frame_id,
                                 'pose': [p.pose.position.x, p.pose.position.y, angle(p.pose.orientation)]}), 10)
    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                             lambda msg: data['status'].append({'t': now(), 'states': [
                                 {'id': bytes(s.goal_info.goal_id.uuid).hex(), 'status': s.status}
                                 for s in msg.status_list]}),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def mapping(msg):
        value = json.loads(msg.data)
        if value.get('event') == 'navigation_velocity_mapping':
            data['mapping'].append({'t': now(), **value})
    node.create_subscription(String, '/whill/controller/event', mapping, 50)
    def interface(msg):
        t = now()
        if t-last['interface'] < .05:
            return
        last['interface'] = t
        value = json.loads(msg.data)
        data['interface'].append({'t': t, **{k: value.get(k) for k in (
            'armed', 'serial_connected', 'bluetooth_connected', 'direction', 'keys_down',
            'left_button_held', 'pointer_coordinate', 'command_sequence', 'last_command')}})
    node.create_subscription(String, '/whill/controller/status', interface, 20)
    def scan(msg):
        counts = {'stop': 0, 'body': 0, 'padded': 0}
        if msg.header.frame_id == 'base_link':
            for i, distance in enumerate(msg.ranges):
                if not math.isfinite(distance) or not msg.range_min <= distance <= msg.range_max:
                    continue
                a = msg.angle_min+i*msg.angle_increment
                x, y = distance*math.cos(a), distance*math.sin(a)
                counts['stop'] += int(-.15 < x < .8 and abs(y) < .3)
                counts['body'] += int(-.18 < x < .77 and abs(y) < .28)
                counts['padded'] += int(-.18 < x < .77 and abs(y) < .33)
        data['scan'].append({'t': now(), 'frame': msg.header.frame_id, **counts})
    node.create_subscription(LaserScan, '/wheelchair/scan', scan, qos_profile_sensor_data)
    started = time.monotonic()
    try:
        while time.monotonic()-started < (float(sys.argv[2]) if len(sys.argv) > 2 else 45):
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        output.write_text(json.dumps(data))
        print(json.dumps({'file': str(output), 'counts': {k: len(v) for k, v in data.items()}}), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
