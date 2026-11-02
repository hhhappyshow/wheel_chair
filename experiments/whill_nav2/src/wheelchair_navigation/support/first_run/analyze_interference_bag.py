#!/usr/bin/env python3
"""Offline, non-motion comparison of per-lidar near-body point populations."""
from collections import Counter, defaultdict
import sys
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
types = {item.name: item.type for item in reader.get_all_topics_and_types()}
tf = Buffer()
rows = defaultdict(list)
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic not in ('/tf_static', '/mid360_201/points_filtered', '/mid360_202/points_filtered'):
        continue
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == '/tf_static':
        for item in msg.transforms:
            tf.set_transform_static(item, 'bag')
        continue
    points = _cloud_to_base(msg, tf, 'base_link', timeout_sec=0.)
    if points is None:
        continue
    body = (points[:,0]>=-.15)&(points[:,0]<=.80)&(np.abs(points[:,1])<=.275)
    _, pts = classify_ground(points[~body], -.036, .15, .90)
    pts = pts[~small_cluster_mask(pts, .27)]
    regions = {
        'front': (pts[:,0]>=.82)&(pts[:,0]<=.86)&(np.abs(pts[:,1])<=.15),
        'left_side': (pts[:,0]>=.15)&(pts[:,0]<=.70)&(pts[:,1]>=.275)&(pts[:,1]<=.34),
    }
    rows[topic].append({name:int(mask.sum()) for name, mask in regions.items()})
for topic, frames in rows.items():
    print(topic, 'frames', len(frames))
    for region in ('front','left_side'):
        values = [frame[region] for frame in frames]
        print(region, 'hit_frames', sum(v>0 for v in values), 'points', sum(values),
              'distribution', dict(sorted(Counter(values).items())))
