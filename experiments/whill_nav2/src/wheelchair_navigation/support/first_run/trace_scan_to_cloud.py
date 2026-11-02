#!/usr/bin/env python3
"""Read-only bag trace: find per-lidar 3D returns behind near-body scan hits."""
import sys
from collections import Counter
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from wheelchair_hardware_bridge.cloud_to_scan import _cloud_to_base
from wheelchair_hardware_bridge.ground_filter import classify_ground
from wheelchair_hardware_bridge.cluster_filter import small_cluster_mask

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
types = {x.name: x.type for x in reader.get_all_topics_and_types()}
clouds = ['/mid360_201/points_filtered', '/mid360_202/points_filtered']
wanted = {'/tf_static', '/wheelchair/scan', *clouds}
tf = Buffer()
last_clouds = {}
summary = Counter()
sampled = 0
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic not in wanted:
        continue
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == '/tf_static':
        for transform in msg.transforms:
            tf.set_transform_static(transform, 'bag')
        continue
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if topic in clouds:
        points = _cloud_to_base(msg, tf, 'base_link', timeout_sec=0.)
        if points is None:
            continue
        body = (points[:,0]>=-.15)&(points[:,0]<=.80)&(np.abs(points[:,1])<=.275)
        _, points = classify_ground(points[~body], -.036, .15, .90)
        points = points[~small_cluster_mask(points, .27)]
        last_clouds[topic] = (stamp, points)
        continue
    ranges = np.asarray(msg.ranges)
    angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
    x, y = ranges*np.cos(angles), ranges*np.sin(angles)
    zone = np.isfinite(ranges)&(x>=.25)&(x<=.75)&(y>=.27)&(y<=.34)
    if np.count_nonzero(zone) < 3:
        continue
    summary['trigger_scan_messages'] += 1
    scan_pts = np.column_stack((x[zone], y[zone]))
    sources = []
    for source in clouds:
        if source not in last_clouds:
            continue
        cstamp, points = last_clouds[source]
        if abs(stamp-cstamp) > .35 or not len(points):
            continue
        xy_delta = points[:,None,:2] - scan_pts[None,:,:]
        match = np.any(np.sum(xy_delta**2, axis=2) < .015**2, axis=1)
        q = points[match]
        if len(q):
            sources.append((source, round(stamp-cstamp, 3), len(q),
                            np.round(q.min(axis=0), 3).tolist(),
                            np.round(q.max(axis=0), 3).tolist()))
            summary[source] += 1
    if sampled < 8:
        print('scan_stamp', round(stamp, 6), 'scan_xy', np.round(scan_pts, 3).tolist(), 'sources', sources)
        sampled += 1
print('counts', dict(summary))
