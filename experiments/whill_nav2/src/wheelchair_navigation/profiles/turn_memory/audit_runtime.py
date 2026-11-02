#!/usr/bin/env python3
"""Read-only verification of the independent profile and its sensor input."""
import collections
import json
import math
import sys
import time
from pathlib import Path
import rclpy
import yaml
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

rclpy.init()
n=rclpy.create_node('turn_memory_runtime_audit')
mask=yaml.safe_load((Path(__file__).resolve().parent/'memory_scan_filter.yaml').read_text())['scan_to_scan_filter_chain']['ros__parameters']['filter1']['params']
seen={}; counts=collections.Counter(); clients=[]
def record(key,msg):
    seen[key]=msg
    counts[key]+=1
for name in ['global','local']:
    n.create_subscription(Costmap,f'/{name}_costmap/costmap_raw',lambda m,k=name:record(k,m),10)
for name,topic in [('input_scan','/wheelchair/scan'),('memory_scan','/wheelchair/memory_scan')]:
    n.create_subscription(LaserScan,topic,lambda m,k=name:record(k,m),qos_profile_sensor_data)
n.create_subscription(Odometry,'/wheelchair/odom',lambda m:record('odom',m),qos_profile_sensor_data)
n.create_subscription(String,'/whill/controller/status',lambda m:record('interface',m),10)
states={}
for name in ['planner_server','controller_server','smoother_server','behavior_server','bt_navigator',
             'waypoint_follower','velocity_smoother','collision_monitor']:
    c=n.create_client(GetState,'/'+name+'/get_state');clients.append(c)
    states[name]=c.call_async(GetState.Request())
questions={'planner_server':['GridBased.plugin','GridBased.allow_unknown','GridBased.allow_reverse_expansion'],
           'controller_server':['FollowPath.plugin','FollowPath.primary_controller','FollowPath.use_path_orientations',
                                'FollowPath.rotate_to_goal_heading','FollowPath.vx_min','FollowPath.vx_max','FollowPath.wz_max',
                                'FollowPath.closed_loop','FollowPath.max_angular_accel',
                                'general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance'],
           'behavior_server':['behavior_plugins','max_rotational_vel','rotational_acc_lim'],
           'velocity_smoother':['max_accel','max_decel','max_velocity','min_velocity']}
for name in ['global','local']:
    questions[f'{name}_costmap/{name}_costmap']=['plugins','track_unknown_space','footprint',
        'memory_obstacles.plugin','memory_obstacles.scan.topic','memory_obstacles.scan.marking',
        'memory_obstacles.scan.clearing','memory_obstacles.scan.observation_persistence',
        'inflation_layer.inflation_radius']
futures={}
for name,keys in questions.items():
    c=n.create_client(GetParameters,'/'+name+'/get_parameters');clients.append(c)
    futures[name]=c.call_async(GetParameters.Request(names=keys))
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
report={'states':{k:f.result().current_state.label if f.done() and f.result() else 'missing' for k,f in states.items()},
        'parameters':{k:dict(zip(questions[k],map(parameter_value_to_python,f.result().values))) for k,f in futures.items() if f.done() and f.result()},
        'received':dict(counts),'maps':{},'scans':{}}
for name in ['global','local']:
    if name in seen:
        g=seen[name]
        report['maps'][name]={'frame':g.header.frame_id,'histogram':dict(collections.Counter(map(int,g.data)))}
for name in ['input_scan','memory_scan']:
    if name in seen:
        s=seen[name];inside=0;finite=0
        for i,d in enumerate(s.ranges):
            if not math.isfinite(d):continue
            finite+=1;a=s.angle_min+i*s.angle_increment;x=d*math.cos(a);y=d*math.sin(a)
            inside+=int(mask['min_x']<x<mask['max_x'] and mask['min_y']<y<mask['max_y'])
        report['scans'][name]={'frame':s.header.frame_id,'finite':finite,'inside_self_echo_mask':inside}
if 'odom' in seen:report['odom_z']=seen['odom'].pose.pose.position.z
if 'interface' in seen:
    value=json.loads(seen['interface'].data)
    report['interface']={k:value.get(k) for k in ['armed','serial_connected','bluetooth_connected','direction']}
expected_allow_unknown=yaml.safe_load((Path(__file__).resolve().parent/'navigation.yaml').read_text())['planner_server']['ros__parameters']['GridBased']['allow_unknown']
report['pass']=(all(x=='active' for x in report['states'].values()) and counts['memory_scan']>5
                and report['scans'].get('memory_scan',{}).get('inside_self_echo_mask')==0
                and report['parameters']['planner_server'].get('GridBased.allow_unknown') is expected_allow_unknown
                and report['parameters']['local_costmap/local_costmap'].get('track_unknown_space') is False
                and report['maps'].get('local',{}).get('histogram',{}).get(255,0)==0)
with Path(sys.argv[1]).open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report),flush=True)
n.destroy_node();rclpy.shutdown()
if not report['pass']:raise SystemExit(1)
