#!/usr/bin/env python3
"""Compare near-robot planning-map cells and costmap cells without publishing."""
import math
import sys
import time
from collections import Counter
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException


class Probe(Node):
    def __init__(self):
        super().__init__('map_layer_diagnostic')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.grids = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ('/wheelchair/planning_map', '/local_costmap/costmap'):
            self.create_subscription(OccupancyGrid, topic,
                                     lambda msg, t=topic: self.grids.__setitem__(t, msg), qos)

rclpy.init()
n = Probe()
end = time.monotonic() + 6
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=.1)
try:
    t = n.tf.lookup_transform('odom', 'base_link', Time()).transform
    q = t.rotation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    print('base odom', round(t.translation.x, 3), round(t.translation.y, 3), round(yaw, 3))
    for topic, msg in n.grids.items():
        grid = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        iy, ix = np.indices(grid.shape)
        x = msg.info.origin.position.x+(ix+.5)*msg.info.resolution-t.translation.x
        y = msg.info.origin.position.y+(iy+.5)*msg.info.resolution-t.translation.y
        bx, by = c*x+s*y, -s*x+c*y
        region = (bx>-.55)&(bx<1.05)&(np.abs(by)<.38)
        values = grid[region]
        print(topic, 'age', round(n.get_clock().now().nanoseconds/1e9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9), 3),
              'counts', dict(sorted(Counter(values.tolist()).items())))
        occ = region & (grid >= 99)
        print('occupied_cell_samples_base', np.round(np.column_stack((bx[occ], by[occ]))[:25], 3).tolist())
        if len(sys.argv) == 3:
            gx, gy = float(sys.argv[1]), float(sys.argv[2])
            col = int(math.floor((gx-msg.info.origin.position.x)/msg.info.resolution))
            row = int(math.floor((gy-msg.info.origin.position.y)/msg.info.resolution))
            if 0 <= row < msg.info.height and 0 <= col < msg.info.width:
                patch = grid[max(0,row-5):row+6, max(0,col-5):col+6]
                print('goal_cell', int(grid[row,col]), 'goal_055m_counts', dict(sorted(Counter(patch.ravel().tolist()).items())))
            else:
                print('goal outside grid')
except TransformException as e:
    print('TF unavailable', e)
n.destroy_node()
rclpy.shutdown()
