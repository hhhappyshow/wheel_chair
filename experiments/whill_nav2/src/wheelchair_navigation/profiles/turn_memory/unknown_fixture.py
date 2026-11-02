#!/usr/bin/env python3
"""Isolated unknown-space input for the existing Gazebo navigation test.

Run with ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 unknown_fixture.py \
    --ros-args -p use_sim_time:=true
Pass use_sim_time:=false for wall time. Timers/stamps follow the chosen clock.
Only publishes test map/scan; CollisionMonitor must keep its real simulated scan.
"""

import math
import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class UnknownFixture(Node):
    def __init__(self):
        super().__init__("turn_memory_unknown_fixture")
        # Node's TimeSource accepts the standard use_sim_time ROS parameter.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid, "/turn_memory_test/unknown_map", map_qos
        )
        self.scan_pub = self.create_publisher(
            LaserScan, "/turn_memory_test/no_returns", qos_profile_sensor_data
        )
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = "odom"
        self.grid.info.width = self.grid.info.height = 200
        self.grid.info.resolution = 0.05
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -5.0
        self.grid.info.origin.orientation.w = 1.0
        cells = [-1] * (200 * 200)
        # World (2.025, 2.025) lies at the centre of cell (140, 140).
        cells[140 * 200 + 140] = 100
        self.grid.data = cells

        self.scan = LaserScan()
        self.scan.header.frame_id = "base_link"
        self.scan.angle_min = -math.pi
        self.scan.angle_increment = 2.0 * math.pi / 720
        self.scan.angle_max = self.scan.angle_min + 719 * self.scan.angle_increment
        self.scan.range_min = 0.0
        self.scan.range_max = 8.0
        self.scan.scan_time = 0.1
        self.scan.time_increment = 0.0
        self.scan.ranges = [math.nan] * 720
        self.map_timer = self.create_timer(0.5, self.publish_map)
        self.scan_timer = self.create_timer(0.1, self.publish_scan)

    def publish_map(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.grid)

    def publish_scan(self):
        self.scan.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(self.scan)


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "198":
        raise SystemExit("Requires ROS_DOMAIN_ID=198; refusing to initialize ROS.")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Requires ROS_LOCALHOST_ONLY=1; refusing to initialize ROS.")
    rclpy.init()
    node = None
    try:
        node = UnknownFixture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
