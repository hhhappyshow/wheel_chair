#!/usr/bin/env python3
"""Request only a plan 0.5 m in front of the rear axle; never command velocity."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node('shaki_stationary_plan_probe')
data = {}
def save(name, msg):
    data[name] = msg
node.create_subscription(Odometry, '/wheelchair/odom', lambda m: save('odom', m), qos_profile_sensor_data)
node.create_subscription(LaserScan, '/wheelchair/scan', lambda m: save('scan', m), qos_profile_sensor_data)
node.create_subscription(OccupancyGrid, '/wheelchair/planning_map', lambda m: save('map', m),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE))
end = time.monotonic() + 8
while time.monotonic() < end and len(data) < 3:
    rclpy.spin_once(node, timeout_sec=0.1)
if len(data) < 3:
    raise RuntimeError(f'missing inputs: {set(("odom", "scan", "map")) - data.keys()}')
now = node.get_clock().now().nanoseconds / 1e9
for name, msg in data.items():
    age = now - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9
    print(f'{name}: frame={msg.header.frame_id}, age={age:.3f}s', flush=True)
    if age < -0.05 or age > (0.75 if name != 'odom' else 0.3):
        raise RuntimeError(f'{name} is stale or in the future')
scan = data['scan']
pts = [(r * math.cos(scan.angle_min + i * scan.angle_increment),
        r * math.sin(scan.angle_min + i * scan.angle_increment))
       for i, r in enumerate(scan.ranges) if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
stop = [(x, y) for x, y in pts if -0.30 < x < 1.05 and abs(y) < 0.38]
front = [x for x, y in pts if x > 0 and abs(y) < 0.38]
print(f'scan stop zone points={len(stop)}, nearest forward x={min(front) if front else None}', flush=True)
print('stop zone xy:', [(round(x, 3), round(y, 3)) for x, y in stop], flush=True)
odom = data['odom']
q = odom.pose.pose.orientation
heading = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
goal = PoseStamped()
goal.header.frame_id = 'odom'
goal.header.stamp = node.get_clock().now().to_msg()
goal.pose.position.x = odom.pose.pose.position.x + 0.5 * math.cos(heading)
goal.pose.position.y = odom.pose.pose.position.y + 0.5 * math.sin(heading)
goal.pose.orientation = q
print(f'base_link +0.5m -> odom ({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}), yaw {heading:.3f}', flush=True)
client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
if not client.wait_for_server(timeout_sec=5):
    raise RuntimeError('planner action not ready')
request = ComputePathToPose.Goal()
request.goal = goal
request.planner_id = 'GridBased'
future = client.send_goal_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=5)
handle = future.result()
if not handle or not handle.accepted:
    raise RuntimeError('planner request rejected')
future = handle.get_result_async()
rclpy.spin_until_future_complete(node, future, timeout_sec=15)
if not future.done():
    raise RuntimeError('planner request timed out')
result = future.result()
path = result.result.path
print(f'plan status={result.status}, poses={len(path.poses)}', flush=True)
if result.status != 4 or len(path.poses) < 2:
    raise RuntimeError('no valid path')
last = path.poses[-1].pose.position
print(f'plan endpoint error={math.hypot(last.x-goal.pose.position.x, last.y-goal.pose.position.y):.3f}m', flush=True)
node.destroy_node()
rclpy.shutdown()
