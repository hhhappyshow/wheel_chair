#!/usr/bin/env python3
"""One supervised forward Nav2 goal; --check-only never sends a navigation goal."""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument('--check-only', action='store_true')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('shaki_forward_half_meter')
messages = {}
received = {}

def save(name, message):
    messages[name] = message
    received[name] = time.monotonic()

node.create_subscription(Odometry, '/wheelchair/odom', lambda msg: save('odom', msg), qos_profile_sensor_data)
node.create_subscription(LaserScan, '/wheelchair/scan', lambda msg: save('scan', msg), qos_profile_sensor_data)
node.create_subscription(String, '/whill/controller/status', lambda msg: save('status', msg), 10)

def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

def wait(future, timeout):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError('ROS request timed out')
    return future.result()

def check_inputs():
    if any(name not in messages for name in ('odom', 'scan', 'status')):
        raise RuntimeError('missing odometry, scan or interface status')
    now = node.get_clock().now().nanoseconds / 1e9
    for name, max_age in (('odom', 0.3), ('scan', 0.75)):
        header = messages[name].header
        age = now - header.stamp.sec - header.stamp.nanosec / 1e9
        if age < -0.05 or age > max_age:
            raise RuntimeError(f'{name} timestamp age {age:.3f}s')
    if time.monotonic() - received['status'] > 2.5:
        raise RuntimeError('interface status is stale')
    status = json.loads(messages['status'].data)
    for key in ('armed', 'hardware_enabled', 'serial_connected', 'bluetooth_connected', 'fine_hid_supported'):
        if not status.get(key):
            raise RuntimeError(f'interface {key} is not ready')
    if status.get('firmware_test_mode'):
        raise RuntimeError('ESP32 is in test mode')
    scan = messages['scan']
    if scan.header.frame_id != 'base_link':
        raise RuntimeError(f'unexpected scan frame {scan.header.frame_id}')
    inside = 0
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        inside += -0.30 < x < 1.05 and abs(y) < 0.38
    if inside:
        raise RuntimeError(f'{inside} scan points inside stop zone')
    return messages['odom']

handle = None
try:
    spin_for(3.0)
    odom = check_inputs()
    if odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_link':
        raise RuntimeError(f'unexpected odometry frames {odom.header.frame_id} -> {odom.child_frame_id}')
    subscribers = node.get_subscriptions_info_by_topic('/wheelchair/nav2/cmd_vel')
    if len(subscribers) != 1 or subscribers[0].node_name != 'whill_cmd_vel_adapter':
        raise RuntimeError(f'unexpected velocity subscribers: {[s.node_name for s in subscribers]}')
    q = odom.pose.pose.orientation
    heading = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    start_x, start_y = odom.pose.pose.position.x, odom.pose.pose.position.y
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = start_x + 0.5 * math.cos(heading)
    goal.pose.position.y = start_y + 0.5 * math.sin(heading)
    goal.pose.orientation = q
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if not planner.wait_for_server(timeout_sec=5):
        raise RuntimeError('planner unavailable')
    request = ComputePathToPose.Goal()
    request.goal, request.planner_id = goal, 'GridBased'
    plan_handle = wait(planner.send_goal_async(request), 5)
    if not plan_handle.accepted:
        raise RuntimeError('plan rejected')
    plan = wait(plan_handle.get_result_async(), 15)
    if plan.status != 4 or len(plan.result.path.poses) < 2:
        raise RuntimeError('no valid path')
    endpoint = plan.result.path.poses[-1].pose.position
    error = math.hypot(endpoint.x-goal.pose.position.x, endpoint.y-goal.pose.position.y)
    if error > 0.05:
        raise RuntimeError(f'planned endpoint {error:.3f}m from goal; limit 0.05m')
    fresh_odom = check_inputs()
    if math.hypot(fresh_odom.pose.pose.position.x-start_x, fresh_odom.pose.pose.position.y-start_y) > 0.03:
        raise RuntimeError('vehicle moved while planning; retry from its current pose')
    print(f'检查通过: 目标 odom=({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}), '
          f'路径终点误差={error:.3f}m', flush=True)
    if args.check_only:
        print('仅检查，没有发送运动目标。', flush=True)
    else:
        nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        if not nav.wait_for_server(timeout_sec=5):
            raise RuntimeError('navigator unavailable')
        request = NavigateToPose.Goal()
        request.pose = goal
        handle = wait(nav.send_goal_async(request), 5)
        if not handle.accepted:
            raise RuntimeError('navigation goal rejected')
        result = handle.get_result_async()
        deadline = time.monotonic() + 25
        dx = dy = 0.0
        while not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            odom = check_inputs()
            x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
            dx, dy = x-start_x, y-start_y
            if math.hypot(dx, dy) > 0.70 or abs(-dx*math.sin(heading)+dy*math.cos(heading)) > 0.20:
                raise RuntimeError('movement exceeded supervised bounds')
        if not result.done():
            raise RuntimeError('navigation timed out')
        response = result.result()
        spin_for(0.5)
        final_odom = messages['odom']
        final_pose = final_odom.pose.pose
        final_q = final_pose.orientation
        final_yaw = math.atan2(2*(final_q.w*final_q.z + final_q.x*final_q.y),
                               1-2*(final_q.y*final_q.y + final_q.z*final_q.z))
        position_error = math.hypot(final_pose.position.x-goal.pose.position.x,
                                    final_pose.position.y-goal.pose.position.y)
        yaw_error = abs(math.atan2(math.sin(final_yaw-heading), math.cos(final_yaw-heading)))
        print(f'导航结束: action status={response.status}; 实际位移={math.hypot(dx, dy):.3f}m; '
              f'目标位置误差={position_error:.3f}m; 朝向误差={math.degrees(yaw_error):.1f}度', flush=True)
        handle = None
        if response.status != 4:
            raise RuntimeError('navigation did not succeed')
        if position_error > 0.10 or yaw_error > math.radians(10):
            raise RuntimeError('post-stop measured goal error exceeds 10cm or 10 degrees')
except (Exception, KeyboardInterrupt) as exc:
    print(f'停止: {exc}', flush=True)
    if handle is not None:
        try:
            wait(handle.cancel_goal_async(), 3)
            print('已请求取消导航；如车未立即停止，请按实体急停。', flush=True)
        except Exception as cancel_error:
            print(f'取消请求失败: {cancel_error}；请立即按实体急停。', flush=True)
    raise SystemExit(1)
finally:
    node.destroy_node()
    rclpy.shutdown()
