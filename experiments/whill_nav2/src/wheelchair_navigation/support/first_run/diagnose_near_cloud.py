#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from wheelchair_hardware_bridge.cloud_to_scan import _cloud_to_base
from wheelchair_hardware_bridge.cluster_filter import small_cluster_mask
from wheelchair_hardware_bridge.ground_filter import classify_ground
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


class Probe(Node):
    def __init__(self):
        super().__init__('near_cloud_diagnostic')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.clouds = {}
        self.scans = []
        for sensor in ('201', '202'):
            self.create_subscription(PointCloud2, f'/mid360_{sensor}/points_filtered',
                                     lambda msg, s=sensor: self.clouds.__setitem__(s, msg), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/wheelchair/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        a = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        r = np.array(msg.ranges)
        x, y = r * np.cos(a), r * np.sin(a)
        mask = np.isfinite(r) & (x >= -.55) & (x <= 1.05) & (np.abs(y) <= .38)
        self.scans.append(np.column_stack((x[mask], y[mask])))

rclpy.init()
n = Probe()
end = time.monotonic() + 6
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=.1)
print('scan_frames', len(n.scans), 'zone_counts', [len(s) for s in n.scans[-20:]])
for s in n.scans:
    if len(s) >= 3:
        print('scan_zone_xy', np.round(s, 3).tolist())
for sensor, cloud in n.clouds.items():
    pts = _cloud_to_base(cloud, n.tf, 'base_link')
    if pts is None:
        print(sensor, 'no transform')
        continue
    body = (pts[:,0]>=-.15)&(pts[:,0]<=.80)&(np.abs(pts[:,1])<=.275)
    _, pts = classify_ground(pts[~body], -.036, .15, .90)
    keep = ~small_cluster_mask(pts, .27)
    zone = (pts[:,0]>=-.55)&(pts[:,0]<=1.05)&(np.abs(pts[:,1])<=.38)
    p = pts[keep & zone]
    print(sensor, 'retained_zone_points', len(p))
    if len(p):
        pairs = cKDTree(p).query_pairs(.05, output_type='ndarray')
        graph = coo_matrix((np.ones(len(pairs)), (pairs[:,0],pairs[:,1])), shape=(len(p),len(p)))
        _, labels = connected_components(graph, directed=False)
        for label in sorted(set(labels), key=lambda k: -np.sum(labels == k))[:8]:
            q = p[labels == label]
            print('cluster', len(q), 'min', np.round(q.min(axis=0),3).tolist(), 'max', np.round(q.max(axis=0),3).tolist())
n.destroy_node()
rclpy.shutdown()
