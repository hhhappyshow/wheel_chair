#!/usr/bin/env python3
"""Read-only live trace of occupied cells near the wheelchair front."""
from collections import Counter, deque
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException
from wheelchair_hardware_bridge.cloud_to_scan import _cloud_to_base


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


class Trace(Node):
    def __init__(self):
        super().__init__('trace_live_front')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scans = deque(maxlen=40)
        self.clouds = {'201': deque(maxlen=20), '202': deque(maxlen=20)}
        self.counts = Counter()
        self.reports = []
        self.previous = -1.
        self.create_subscription(LaserScan, '/wheelchair/scan', self.scans.append, qos_profile_sensor_data)
        for sensor in self.clouds:
            self.create_subscription(PointCloud2, f'/mid360_{sensor}/points_filtered',
                                     lambda m, s=sensor: self.clouds[s].append(m), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/wheelchair/planning_map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_map(self, msg):
        self.counts['maps'] += 1
        try:
            pose = self.tf.lookup_transform('odom', 'base_link', Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            self.counts['tf_missing'] += 1
            return
        q = pose.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        grid = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        iy, ix = np.nonzero(grid == 100)
        dx = msg.info.origin.position.x+(ix+.5)*msg.info.resolution-pose.translation.x
        dy = msg.info.origin.position.y+(iy+.5)*msg.info.resolution-pose.translation.y
        bx, by = c*dx+s*dy, -s*dx+c*dy
        near = (bx>=.70)&(bx<=1.2)&(np.abs(by)<=.30)
        if not near.any():
            return
        self.counts['front_occupied_maps'] += 1
        if stamp(msg)-self.previous < .4:
            return
        self.previous = stamp(msg)
        cells = np.column_stack((bx[near], by[near]))
        cell_keys = set(zip(ix[near].tolist(), iy[near].tolist()))
        scans = [m for m in self.scans if -.05 <= stamp(msg)-stamp(m) <= .50]
        evidence = []
        matching_scan_points = []
        for scan in scans:
            try:
                scan_pose = self.tf.lookup_transform('odom', 'base_link', Time.from_msg(scan.header.stamp)).transform
            except TransformException:
                continue
            sq = scan_pose.rotation
            scan_yaw = math.atan2(2*(sq.w*sq.z+sq.x*sq.y), 1-2*(sq.y*sq.y+sq.z*sq.z))
            sc, ss = math.cos(scan_yaw), math.sin(scan_yaw)
            a = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
            r = np.asarray(scan.ranges)
            good = np.isfinite(r)&(r<2.)
            pts = np.column_stack((r[good]*np.cos(a[good]), r[good]*np.sin(a[good])))
            if not len(pts):
                continue
            wx = scan_pose.translation.x + sc*pts[:,0]-ss*pts[:,1]
            wy = scan_pose.translation.y + ss*pts[:,0]+sc*pts[:,1]
            cx = np.floor((wx-msg.info.origin.position.x)/msg.info.resolution).astype(int)
            cy = np.floor((wy-msg.info.origin.position.y)/msg.info.resolution).astype(int)
            selfhit = (pts[:,0]>=-.38)&(pts[:,0]<=.82)&(np.abs(pts[:,1])<=.35)
            match = np.array([tuple(k) in cell_keys for k in zip(cx,cy)]) & ~selfhit
            if match.any():
                evidence.append({'age':round(stamp(msg)-stamp(scan),3),
                                 'xy':np.round(pts[match][:12],3).tolist()})
                matching_scan_points.extend(pts[match].tolist())
        sources = {}
        for sensor, queue in self.clouds.items():
            if not queue:
                continue
            cloud = min(queue, key=lambda m: abs(stamp(msg)-stamp(m)))
            age = stamp(msg)-stamp(cloud)
            if abs(age)>.5:
                continue
            pts = _cloud_to_base(cloud, self.tf, 'base_link', timeout_sec=0.)
            if pts is None or len(pts)==0:
                continue
            if not matching_scan_points:
                continue
            match = np.min(np.linalg.norm(pts[:,None,:2]-np.asarray(matching_scan_points)[None,:,:],axis=2),axis=1)<.015
            pts = pts[match]
            if len(pts):
                sources[sensor] = {'age':round(age,3),'n':len(pts),
                                   'min':np.round(pts.min(axis=0),3).tolist(),
                                   'max':np.round(pts.max(axis=0),3).tolist()}
        self.reports.append({'stamp':round(stamp(msg),3), 'occupied_xy':np.round(cells,3).tolist(),
                             'scan_matches':evidence[-3:], 'cloud_sources':sources})


rclpy.init()
node = Trace()
end = time.monotonic()+20
while time.monotonic()<end:
    rclpy.spin_once(node, timeout_sec=.1)
print('counts', dict(node.counts))
for report in node.reports[:15]:
    print(report)
node.destroy_node()
rclpy.shutdown()
