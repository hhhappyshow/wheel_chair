"""Receive actual D555 RGB/depth frames with one owned official ROS driver.

No controller, network or clock changes. Raw depth is saved without inventing
its metric scale or a base-frame transform. A run-owned camera session can
keep the driver alive between captures; standalone capture stops its driver.
"""
import argparse
from collections import OrderedDict
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def driver_configuration():
    params={'camera_name':'d555_check','enable_color':True,'enable_color1':False,'enable_depth':True,
        'enable_infra':False,'enable_infra1':False,'enable_infra2':False,'enable_gyro':False,
        'enable_accel':False,'enable_motion':False,'enable_sync':True,'align_depth.enable':True,
        'rgb_camera.color_profile':'640,360,15','depth_module.depth_profile':'640,360,15',
        'rgb_camera.color_format':'RGB8','pointcloud.enable':False,'publish_tf':True,'initial_reset':False}
    command=['ros2','run','realsense2_camera','realsense2_camera_node','--ros-args',
        '-r','__ns:=/s20_deploy_readonly','-r','__node:=d555_check']
    for key,value in params.items():
        command+=['-p',f'{key}:={str(value).lower() if isinstance(value,bool) else value}']
    return params,command


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=35.)
    parser.add_argument('--snapshot-joints',action='store_true',help='Read existing SDK joints only; no motion or outputs')
    parser.add_argument('--controller-snapshot-output',type=Path)
    parser.add_argument('--frames',type=int,default=0,help='Finish after this many RGB frames; zero uses the full duration')
    parser.add_argument('--camera-session',type=Path,help='Use this run-owned persistent driver; do not restart it')
    parser.add_argument('--acquisition-s',type=float,default=3.,help='Maximum collection window after the first paired frame; startup is separate')
    a=parser.parse_args(argv);a.output.mkdir(parents=True,exist_ok=False)
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from sensor_msgs.msg import Image,CameraInfo
    from tf2_msgs.msg import TFMessage
    from ros_observation_bridge import image_array
    rclpy.init();node=rclpy.create_node('s20_camera_receive')
    root='/s20_deploy_readonly/d555_check'
    counts={'rgb':0,'depth':0,'info':0};first={};latest={};driver_transforms={}
    paired={};pending={'rgb':OrderedDict(),'depth':OrderedDict()};joint_samples=[];first_pair_s=None
    def static_transforms(message):
        for t in message.transforms:
            driver_transforms[t.child_frame_id]=dict(parent=t.header.frame_id,child=t.child_frame_id,
                translation_m=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                quaternion_xyzw=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])
    def receive(kind,msg):
        nonlocal first_pair_s
        counts[kind]+=1
        if kind not in first:first[kind]=msg
        latest[kind]=msg
        if kind in pending:
            stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            pending[kind][stamp]=msg
            while len(pending[kind])>30:pending[kind].popitem(last=False)
            common=pending['rgb'].keys() & pending['depth'].keys()
            if common:
                stamp=max(common)
                if first_pair_s is None:first_pair_s=time.monotonic()
                paired.update(rgb=pending['rgb'][stamp],depth=pending['depth'][stamp],
                              received_monotonic_s=time.monotonic(),stamp_ns=stamp)
    subscriptions=[node.create_subscription(Image,root+'/color/image_raw',lambda m:receive('rgb',m),qos_profile_sensor_data),
        node.create_subscription(Image,root+'/aligned_depth_to_color/image_raw',lambda m:receive('depth',m),qos_profile_sensor_data),
        node.create_subscription(CameraInfo,root+'/aligned_depth_to_color/camera_info',lambda m:receive('info',m),qos_profile_sensor_data)]
    subscriptions.append(node.create_subscription(TFMessage,'/tf_static',static_transforms,
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
    sdk=None;last_joint=0.
    if a.snapshot_joints:
        from jetson_interfaces import SDKInterfaces
        sdk=SDKInterfaces();sdk.__enter__()
        if a.controller_snapshot_output:
            a.controller_snapshot_output.write_text(json.dumps(sdk.snapshot(),indent=2))
    params,command=driver_configuration()
    driver=None;log=None
    if a.camera_session:
        from camera_session import owned_driver_pid
        driver_pid=owned_driver_pid(a.camera_session)
        driver_log=a.camera_session/'driver.log'
    else:
        driver_log=a.output/'driver.log'
        log=driver_log.open('w')
        driver=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        driver_pid=driver.pid
    began=time.monotonic()
    collection_finished_s=began
    try:
        while time.monotonic()-began<a.duration:
            if driver is not None and driver.poll() is not None:break
            rclpy.spin_once(node,timeout_sec=.1)
            if sdk is not None and time.monotonic()-last_joint>=.1:
                start=time.monotonic();q=list(map(float,sdk.read('HRIF_ReadActJointPos',0)[:6]))
                last_joint=time.monotonic()
                joint_samples.append(dict(q_deg=q,read_midpoint_monotonic_s=(start+last_joint)/2))
            if a.frames and counts['rgb']>=a.frames and paired and 'info' in latest:break
            if first_pair_s is not None and time.monotonic()-first_pair_s>=a.acquisition_s:break
        collection_finished_s=time.monotonic()
    finally:
        if driver is not None and driver.poll() is None:
            os.killpg(driver.pid,signal.SIGINT)
            try:driver.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(driver.pid,signal.SIGTERM);driver.wait(timeout=5)
        if log is not None:log.close()
        if sdk is not None:sdk.__exit__(None,None,None)
        node.destroy_node();rclpy.shutdown()
    result=dict(scope=__doc__,elapsed_s=time.monotonic()-began,counts=counts,
        startup_wait_s=first_pair_s-began if first_pair_s is not None else None,
        acquisition_s=collection_finished_s-first_pair_s if first_pair_s is not None else None,
        acquisition_budget_s=a.acquisition_s,
        driver_exit_code=driver.returncode if driver is not None else None,
        driver_session=str(a.camera_session) if a.camera_session else None,
        driver_pid=driver_pid,parameters=params,motion_sent=False,io_written=False,
        metric_depth_scale_verified=False,base_camera_tf_verified=False)
    result['driver_static_transforms']=list(driver_transforms.values())
    result['joint_samples']=joint_samples
    if 'RealSense ROS v4.58.4' in driver_log.read_text():
        result['ros_depth_unit_m']=.001
        result['depth_unit_basis']='realsense2_camera4.58.4 fillCVMatImageAndReturnStatus -> fix_depth_scale, normalized millimetres'
        result['depth_unit_source']='https://raw.githubusercontent.com/realsenseai/realsense-ros/4.58.4/realsense2_camera/src/base_realsense_node.cpp'
    for kind,msg in first.items():
        result[kind]=dict(width=msg.width,height=msg.height,frame=msg.header.frame_id,
            stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        if kind=='info':result[kind].update(k=list(msg.k),d=list(msg.d),distortion_model=msg.distortion_model)
        else:
            result[kind]['encoding']=msg.encoding
            array=image_array(msg,depth=kind=='depth')
            if kind=='rgb':cv2.imwrite(str(a.output/'rgb.png'),array)
            else:np.savez_compressed(a.output/'depth_raw.npz',depth_raw_units=array)
    for kind,msg in latest.items():
        if kind=='info':continue
        array=image_array(msg,depth=kind=='depth')
        if kind=='rgb':
            cv2.imwrite(str(a.output/'rgb_last.png'),array)
            result['rgb_last']=dict(stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                mean_intensity=float(np.mean(array)),max_intensity=int(np.max(array)))
        else:
            np.savez_compressed(a.output/'depth_last_raw.npz',depth_raw_units=array)
            result['depth_last']=dict(nonzero_fraction=float(np.count_nonzero(array)/array.size),
                min_raw=int(array.min()),max_raw=int(array.max()))
    if paired:
        cv2.imwrite(str(a.output/'rgb_paired.png'),image_array(paired['rgb']))
        np.savez_compressed(a.output/'depth_paired_raw.npz',depth_raw_units=image_array(paired['depth'],depth=True))
        result['paired']=dict(stamp_ns=paired['stamp_ns'],received_monotonic_s=paired['received_monotonic_s'],
            rgb_frame=paired['rgb'].header.frame_id,depth_frame=paired['depth'].header.frame_id,
            encoding=paired['depth'].encoding)
        if joint_samples:
            nearest=min(joint_samples,key=lambda sample:abs(sample['read_midpoint_monotonic_s']-paired['received_monotonic_s']))
            result['paired']['nearest_received_joint_sample']=nearest
            result['paired']['receipt_joint_delta_s']=nearest['read_midpoint_monotonic_s']-paired['received_monotonic_s']
            result['paired']['synchronization']='Nearest SDK read to local RGB-D receipt, not calibrated sensor acquisition synchronization'
    (a.output/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(output=str(a.output),counts=counts,paired=result.get('paired'),
        ros_depth_unit_m=result.get('ros_depth_unit_m'),motion_sent=False,io_written=False),indent=2))


if __name__=='__main__':main()
