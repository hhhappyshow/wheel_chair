#!/usr/bin/env python3
"""Read-only per-sensor assessment of near-body returns in a recorded bag."""
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

bag = sys.argv[1]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
types = {x.name: x.type for x in reader.get_all_topics_and_types()}
wanted = {'/tf_static', '/mid360_201/points_filtered', '/mid360_202/points_filtered'}
buffer = Buffer()
results = defaultdict(list)
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic not in wanted:
        continue
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == '/tf_static':
        for transform in msg.transforms:
            buffer.set_transform_static(transform, 'bag')
        continue
    pts = _cloud_to_base(msg, buffer, 'base_link', timeout_sec=0.)
    if pts is None:
        continue
    body = (pts[:,0]>=-.15)&(pts[:,0]<=.80)&(np.abs(pts[:,1])<=.275)
    _, obs = classify_ground(pts[~body], -.036, .15, .90)
    suppressed = small_cluster_mask(obs, .27)
    obs = obs[~suppressed]
    side = (obs[:,0]>=.15)&(obs[:,0]<=.70)&(obs[:,1]>=.275)&(obs[:,1]<=.34)
    p = obs[side]
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
    results[topic].append((stamp, len(p), np.round(p.mean(axis=0), 3).tolist() if len(p) else None))
for topic, rows in results.items():
    stamps = [x[0] for x in rows]
    print(topic, 'frames', len(rows), 'unique_stamps', len(set(stamps)),
          'near_side_count_distribution', dict(sorted(Counter(x[1] for x in rows).items())))
    print('sample_nonempty', [r for r in rows if r[1]][:15])
