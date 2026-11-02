#!/usr/bin/env python3
"""Trace velocity, odometry and paths only in the isolated Gazebo domain."""

import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts

if os.environ.get('ROS_DOMAIN_ID') != '198' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Requires the isolated simulation domain 198')


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def norm_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def summary(samples):
    if len(samples) < 2:
        return {'samples': len(samples)}
    dt = [b['t']-a['t'] for a, b in zip(samples, samples[1:]) if b['t']-a['t'] > 0.001]
    accel = [(b['v']-a['v'])/(b['t']-a['t'])
             for a, b in zip(samples, samples[1:]) if 0.02 <= b['t']-a['t'] <= 0.2]
    angular_accel = [(b['w']-a['w'])/(b['t']-a['t'])
                     for a, b in zip(samples, samples[1:]) if 0.02 <= b['t']-a['t'] <= 0.2]
    result = {'samples': len(samples), 'v_min': min(s['v'] for s in samples),
              'v_max': max(s['v'] for s in samples), 'w_min': min(s['w'] for s in samples),
              'w_max': max(s['w'] for s in samples)}
    if dt:
        result['median_hz'] = 1.0 / statistics.median(dt)
        result['max_gap_s'] = max(dt)
    if accel:
        result['max_abs_linear_accel'] = max(abs(v) for v in accel)
        result['max_abs_angular_accel'] = max(abs(v) for v in angular_accel)
    return result


class Trace(Node):
    def __init__(self):
        super().__init__('hybrid_sim_telemetry')
        self.series = {'raw': [], 'smooth': [], 'drive': [], 'odom': [], 'plan': []}
        self.latest_odom = None
        self.contacts = []
        for name, topic in [('raw', '/wheelchair/nav2/cmd_vel_raw'),
                            ('smooth', '/wheelchair/nav2/cmd_vel_smoothed'),
                            ('drive', '/wheelchair/sim/drive')]:
            self.create_subscription(Twist, topic, lambda m, k=name: self.twist(k, m), 10)
        self.create_subscription(Odometry, '/wheelchair/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self.plan, 10)
        self.create_subscription(Contacts, '/wheelchair/sim/contacts', self.on_contact,
                                 qos_profile_sensor_data)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def twist(self, name, msg):
        self.series[name].append({'t': self.now(), 'wall_t': time.monotonic(), 'v': msg.linear.x, 'w': msg.angular.z})

    def odom(self, msg):
        p = msg.pose.pose
        sample = {'t': self.now(), 'stamp': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                  'x': p.position.x, 'y': p.position.y, 'yaw': yaw(p),
                  'v': msg.twist.twist.linear.x, 'w': msg.twist.twist.angular.z}
        self.latest_odom = sample
        self.series['odom'].append(sample)

    def plan(self, msg):
        self.series['plan'].append({
            't': self.now(),
            'poses': [[p.pose.position.x, p.pose.position.y, yaw(p.pose)] for p in msg.poses],
        })

    def on_contact(self, msg):
        if msg.contacts:
            self.contacts.append({'t': self.now(), 'count': len(msg.contacts)})

    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not predicate():
            rclpy.spin_once(self, timeout_sec=0.05)
        return predicate()

    def run_goal(self, label, dx, dy, dyaw):
        start = dict(self.latest_odom)
        c, s = math.cos(start['yaw']), math.sin(start['yaw'])
        target_x = start['x'] + dx*c - dy*s
        target_y = start['y'] + dx*s + dy*c
        target_yaw = norm_angle(start['yaw'] + dyaw)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'odom'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target_x
        goal.pose.pose.position.y = target_y
        goal.pose.pose.orientation.z = math.sin(target_yaw / 2)
        goal.pose.pose.orientation.w = math.cos(target_yaw / 2)
        begin = self.now()
        begin_wall = time.monotonic()
        sent = self.client.send_goal_async(goal)
        if not self.spin_until(sent.done, 10) or not sent.result().accepted:
            return {'case': label, 'error': 'goal rejected'}
        handle = sent.result()
        future = handle.get_result_async()
        if not self.spin_until(future.done, 50):
            cancel = handle.cancel_goal_async()
            self.spin_until(cancel.done, 5)
            canceled = self.spin_until(future.done, 8)
            return {'case': label, 'error': 'timeout', 'cancel_completed': canceled, 'start': start,
                    'target': [target_x, target_y, target_yaw],
                    'end': dict(self.latest_odom)}
        end = self.now()
        finish = dict(self.latest_odom)
        result = {'case': label, 'status': future.result().status,
                  'duration_s': end-begin,
                  'wall_duration_s': time.monotonic()-begin_wall,
                  'xy_error_m': math.hypot(finish['x']-target_x, finish['y']-target_y),
                  'yaw_error_rad': norm_angle(finish['yaw']-target_yaw),
                  'final_v': finish['v'], 'final_w': finish['w'],
                  'start': start, 'target': [target_x, target_y, target_yaw], 'end': finish}
        result['strict_latest_odom_criteria'] = (
            result['xy_error_m'] <= 0.15 and abs(result['yaw_error_rad']) <= math.radians(10)
            and abs(result['final_v']) <= 0.05 and abs(result['final_w']) <= 0.10)
        for key, samples in self.series.items():
            subset = [sample for sample in samples if begin <= sample['t'] <= end]
            result[key] = {'samples': len(subset)} if key == 'plan' else summary(subset)
        result['contacts'] = sum(v['count'] for v in self.contacts if begin <= v['t'] <= end)
        return result


def main():
    output = Path(sys.argv[1])
    if output.exists():
        raise RuntimeError(f'Refusing to overwrite {output}')
    rclpy.init()
    node = Trace()
    results = []
    try:
        if not node.client.wait_for_server(timeout_sec=30):
            raise RuntimeError('Navigation action unavailable')
        if not node.spin_until(lambda: node.latest_odom is not None and node.now() > 0, 20):
            raise RuntimeError('Simulation odometry/clock unavailable')
        cases = [('forward', 0.6, 0.0, 0.0), ('reverse', -0.6, 0.0, 0.0),
                 ('right_front', 0.55, -0.4, -0.45),
                 ('right_front_clear', 0.35, -0.35, -0.45),
                 ('right_front_padded_clear', 0.25, -0.35, -0.45),
                 ('left_turn', 0.0, 0.0, 0.6),
                 ('right_turn', 0.0, 0.0, -0.6), ('u_turn', 0.0, 0.0, math.pi),
                 ('depart_right_angle', 0.0, -0.6, -math.pi/2),
                 ('reverse_after_turn', -0.6, 0.0, 0.0)]
        selected = os.environ.get('HYBRID_SIM_CASES')
        if selected:
            names = set(selected.split(','))
            cases = [case for case in cases if case[0] in names]
            if not cases or names != {case[0] for case in cases}:
                raise RuntimeError('Unknown simulation case selection')
        for case in cases:
            result = node.run_goal(*case)
            results.append(result)
            print(json.dumps(result), flush=True)
            if result.get('contacts') or result.get('cancel_completed') is False:
                break
            # Let canceled/completed commands decay before the next independent request.
            settle = time.monotonic()+2
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=.05)
    finally:
        output.write_text(json.dumps({'results': results, 'series': node.series,
                                      'contacts': node.contacts}, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    if len(results) != len(cases) or any(
            r.get('status') != GoalStatus.STATUS_SUCCEEDED or r.get('contacts')
            or not r.get('strict_latest_odom_criteria') for r in results):
        raise RuntimeError('Not all simulation cases passed; see trace file')


if __name__ == '__main__':
    main()
