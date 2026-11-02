#!/usr/bin/env python3
"""Compare raw Livox metadata of near-body candidates with other returns."""
from collections import Counter
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from wheelchair_hardware_bridge.cloud_to_scan import _cloud_to_base

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {item.name:item.type for item in reader.get_all_topics_and_types()}
tf = Buffer()
tags = Counter()
near_tags = Counter()
near_intensity = []
near_frames = 0
intensity_counts = Counter()
zero_away_tags = Counter()
for _ in iter(int, 1):
    if not reader.has_next():
        break
    topic, data, _ = reader.read_next()
    if topic not in ('/tf_static', '/mid360_202/points_filtered'):
        continue
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == '/tf_static':
        for transform in msg.transforms:
            tf.set_transform_static(transform, 'bag')
        continue
    fields = {f.name:f for f in msg.fields}
    if not {'x','y','z','tag','intensity'} <= fields.keys() or msg.height != 1:
        continue
    endian = '>' if msg.is_bigendian else '<'
    names = ['x','y','z','intensity','tag']
    dtype = np.dtype({'names': names, 'formats': [endian+'f4']*4+['u1'],
                      'offsets': [fields[n].offset for n in names], 'itemsize': msg.point_step})
    rec = np.frombuffer(msg.data, dtype=dtype, count=msg.width)
    finite = np.isfinite(np.stack([rec[n] for n in ['x','y','z']], axis=1)).all(axis=1)
    rec = rec[finite]
    xyz = _cloud_to_base(msg, tf, 'base_link', timeout_sec=0.)
    if xyz is None or len(xyz) != len(rec):
        continue
    tags.update(rec['tag'].tolist())
    intensity_counts['all'] += len(rec)
    intensity_counts['zero'] += np.count_nonzero(rec['intensity'] == 0)
    mask = (xyz[:,0]>=.33)&(xyz[:,0]<=.43)&(xyz[:,1]>=.275)&(xyz[:,1]<=.31)&(xyz[:,2]>=.30)&(xyz[:,2]<=.36)
    if mask.sum() >= 4:
        near_frames += 1
        near_tags.update(rec['tag'][mask].tolist())
        near_intensity += rec['intensity'][mask].tolist()
    away = (np.hypot(xyz[:,0], xyz[:,1]) > 1.) & (rec['intensity'] == 0)
    zero_away_tags.update(rec['tag'][away].tolist())
print('near_frames', near_frames, 'near_tag', dict(near_tags), 'near_intensity', near_intensity)
print('all_point_tags', dict(tags))
print('all_intensity_counts', dict(intensity_counts), 'zero_intensity_away_tags', dict(zero_away_tags))
