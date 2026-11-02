#!/usr/bin/env python3
"""Read-only comparison of scan hits with previous odom-frame observations."""
from collections import deque
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


class Shadow(Node):
    def __init__(self):
        super().__init__('shadow_scan_consistency')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.history = deque(maxlen=4)
        self.last_stamp = None
        self.results = []
        self.missing_tf = 0
        self.create_subscription(LaserScan, '/wheelchair/scan', self.scan, qos_profile_sensor_data)

    def scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp == stamp:
            return
        self.last_stamp = stamp
        try:
            tf = self.tf.lookup_transform('odom', msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            self.missing_tf += 1
            return
        q = tf.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        r = np.asarray(msg.ranges)
        a = msg.angle_min + np.arange(len(r))*msg.angle_increment
        good = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        x, y = r[good]*np.cos(a[good]), r[good]*np.sin(a[good])
        xy = np.column_stack((x*math.cos(yaw)-y*math.sin(yaw)+tf.translation.x,
                              x*math.sin(yaw)+y*math.cos(yaw)+tf.translation.y))
        zone = (x >= -.55) & (x <= 1.05) & (np.abs(y) <= .38)
        past = [p for t, p in self.history if 0 < stamp-t <= .35 and len(p)]
        hits = np.zeros(len(xy), dtype=int)
        for p in past:
            hits += cKDTree(p).query(xy, distance_upper_bound=.08)[0] < .08
        if past:
            self.results.append((stamp, int(zone.sum()), int((zone & (hits >= 1)).sum()),
                                 int((zone & (hits >= 2)).sum()), len(past)))
        self.history.append((stamp, xy))


rclpy.init()
n = Shadow()
end = time.monotonic() + 20
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=.1)
print('unique_scans', len(n.results), 'missing_tf', n.missing_tf)
for i, label in ((1, 'raw'), (2, 'one_previous'), (3, 'two_previous')):
    print(label, 'zone_trigger_frames', sum(row[i] >= 3 for row in n.results),
          'of', len(n.results))
print('recent_stamp_raw_prev1_prev2_history', n.results[-15:])
n.destroy_node()
rclpy.shutdown()
