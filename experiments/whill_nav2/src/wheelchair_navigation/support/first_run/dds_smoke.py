#!/usr/bin/env python3
"""Isolated inter-process DDS smoke test on a non-robot pointcloud topic."""
import math
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

role = sys.argv[1]
rclpy.init()
node = rclpy.create_node('shaki_dds_' + role)
qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
if role == 'receive':
    latencies = []
    sequences = set()
    def receive(msg):
        age = (node.get_clock().now().nanoseconds -
               msg.header.stamp.sec * 1_000_000_000 - msg.header.stamp.nanosec) / 1e6
        latencies.append(age)
        sequences.add(int(msg.header.frame_id))
    node.create_subscription(PointCloud2, '/shaki_dds_bench/cloud', receive, qos)
    end = time.monotonic() + 10
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not latencies:
        raise RuntimeError('no samples received')
    ordered = sorted(latencies)
    print(f'received={len(latencies)} unique={len(sequences)} '
          f'p50={ordered[len(ordered)//2]:.2f}ms p95={ordered[math.floor((len(ordered)-1)*.95)]:.2f}ms '
          f'max={max(ordered):.2f}ms', flush=True)
elif role == 'send':
    publisher = node.create_publisher(PointCloud2, '/shaki_dds_bench/cloud', qos)
    deadline = time.monotonic()+5
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() < 1:
        raise RuntimeError('subscriber not discovered')
    cloud = PointCloud2()
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 32768, 4, 131072
    cloud.data = bytes(cloud.row_step)
    for sequence in range(40):
        cloud.header.frame_id = str(sequence)
        cloud.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(cloud)
        time.sleep(0.125)
    print('sent=40 payload=128KiB rate=8Hz best_effort', flush=True)
else:
    raise RuntimeError('use send or receive')
node.destroy_node()
rclpy.shutdown()
