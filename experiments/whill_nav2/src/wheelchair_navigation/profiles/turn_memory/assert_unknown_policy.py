#!/usr/bin/env python3
"""Assert unknown stays visible globally, is traversable locally, and lethal stays lethal."""
import json
import math
import os
import sys
import time
from pathlib import Path
import rclpy
from nav2_msgs.msg import Costmap
assert os.environ.get('ROS_DOMAIN_ID') == '198' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init()
n = rclpy.create_node('turn_memory_unknown_policy_check')
seen = {}
for name in ['global','local']:
    n.create_subscription(Costmap,f'/{name}_costmap/costmap_raw',lambda m,k=name:seen.__setitem__(k,m),10)
def cells(g):
    md=g.metadata
    def at(x,y):
        ix=math.floor((x-md.origin.position.x)/md.resolution)
        iy=math.floor((y-md.origin.position.y)/md.resolution)
        return int(g.data[iy*md.size_x+ix])
    return {'start':at(0,0), 'reverse_target':at(-.6,0), 'known_obstacle':at(2.025,2.025)}
expected={'global':{'start':255,'reverse_target':255,'known_obstacle':254},
          'local':{'start':0,'reverse_target':0,'known_obstacle':254}}
end=time.monotonic()+12
result={}
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.1)
    result={k:cells(g) for k,g in seen.items()}
    if result==expected:break
with Path(sys.argv[1]).open('x') as f:json.dump({'actual':result,'expected':expected,'pass':result==expected},f,indent=2)
print(json.dumps(result))
n.destroy_node();rclpy.shutdown()
if result!=expected:raise SystemExit(1)
