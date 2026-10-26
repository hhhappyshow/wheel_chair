"""One-shot real ROS2 RGB-D capture; no SDK, camera driver launch or simulation.

Source ROS Humble and the existing official DDS driver's overlay externally.
Topic names, registered RGB/depth grid and target TF frame are explicit inputs.
"""
import argparse
from collections import OrderedDict
from datetime import datetime, timezone
import json
from pathlib import Path
import time


def arguments():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('rgb-topic','depth-topic','camera-info-topic','target-frame','camera-source'):
        p.add_argument('--'+name,required=True)
    p.add_argument('--registered-depth-to-rgb',action='store_true',required=True,
                   help='Operator declaration: depth and K are on the RGB pixel grid; not independently verified here')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--depth-scale-m',type=float,
                   help='Required verified meters/unit for 16UC1; 32FC1 is axial meters')
    p.add_argument('--joint-topic',help='Optional existing measured sensor_msgs/JointState topic')
    p.add_argument('--joint-names',nargs=6)
    p.add_argument('--timeout-s',type=float,default=20.)
    a=p.parse_args()
    if a.timeout_s<=0:p.error('timeout-s must be positive')
    if a.joint_topic and not a.joint_names:p.error('joint-topic requires six joint-names in planner order')
    return a


def stamp_ns(msg):
    return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec


def image_array(msg,depth=False):
    import numpy as np
    if depth:
        kind={'16UC1':'u2','32FC1':'f4'}.get(msg.encoding)
        if kind is None:raise ValueError('Unsupported depth encoding: '+msg.encoding)
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+kind)
        return np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                          strides=(msg.step,dtype.itemsize)).astype(float)
    if msg.encoding not in ('rgb8','bgr8'):raise ValueError('Unsupported color encoding: '+msg.encoding)
    image=np.ndarray((msg.height,msg.width,3),dtype=np.uint8,buffer=msg.data,
                     strides=(msg.step,3,1)).copy()
    return image[:,:,::-1].copy() if msg.encoding=='rgb8' else image


def metric_cloud(z,info,tf):
    import cv2
    import numpy as np
    from scipy.spatial.transform import Rotation
    k=np.asarray(info.k,dtype=float).reshape(3,3)
    v,u=np.indices(z.shape)
    distortion=np.asarray(info.d,dtype=float)
    if distortion.size and np.any(distortion):
        if info.distortion_model not in ('plumb_bob','rational_polynomial'):
            raise ValueError('Unsupported distortion model: '+info.distortion_model)
        xy=cv2.undistortPoints(np.column_stack((u.ravel(),v.ravel())).astype(float)[:,None,:],
                               k,distortion).reshape(*z.shape,2)
        camera=np.dstack((xy*z[:,:,None],z))
    else:
        camera=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)
    q=tf.transform.rotation;t=tf.transform.translation
    matrix=np.eye(4)
    matrix[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    matrix[:3,3]=[t.x,t.y,t.z]
    points=camera@matrix[:3,:3].T+matrix[:3,3]
    points[~np.isfinite(z)|(z<=0)]=np.nan
    return points,matrix


def main():
    a=arguments()
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo,JointState
    from tf2_ros import Buffer,TransformListener
    if not np.isfinite(a.timeout_s) or (a.depth_scale_m is not None and
            (not np.isfinite(a.depth_scale_m) or a.depth_scale_m<=0)):
        raise ValueError('Finite positive timeout/depth scale required')
    a.output.mkdir(parents=True,exist_ok=False)
    report=dict(schema='s20_real_ros_observation/v1',status='NO_OBSERVATION',
        hardware_motion=False,actuator_outputs_written=False,simulator_fallback=False,
        camera_source_declared=a.camera_source,camera_origin_independently_verified=False,
        transport='existing ROS2 sensor_msgs from externally managed official DDS driver',
        topics=dict(rgb=a.rgb_topic,depth=a.depth_topic,camera_info=a.camera_info_topic,joints=a.joint_topic),
        frame_id=a.target_frame,registered_depth_to_rgb_declared=True,
        camera_registration_independently_verified=False,semantic_inference_executed=False)
    rclpy.init();node=rclpy.create_node('s20_real_observation_bridge')
    buffer=Buffer();listener=TransformListener(buffer,node)
    rgb,depth=OrderedDict(),OrderedDict();infos=[];joints=[]
    def remember(cache,msg):
        cache[stamp_ns(msg)]=msg
        while len(cache)>30:cache.popitem(last=False)
    subscriptions=[
        node.create_subscription(Image,a.rgb_topic,lambda m:remember(rgb,m),qos_profile_sensor_data),
        node.create_subscription(Image,a.depth_topic,lambda m:remember(depth,m),qos_profile_sensor_data),
        node.create_subscription(CameraInfo,a.camera_info_topic,lambda m:infos.__setitem__(slice(None),[m]),qos_profile_sensor_data)]
    if a.joint_topic:
        subscriptions.append(node.create_subscription(JointState,a.joint_topic,
            lambda m:joints.__setitem__(slice(None),[m]),qos_profile_sensor_data))
    began=time.monotonic();last_error=None
    try:
        while time.monotonic()-began<a.timeout_s:
            rclpy.spin_once(node,timeout_sec=.05)
            common=sorted(set(rgb)&set(depth))
            if not common or not infos:continue
            stamp=common[-1];color,raw=rgb[stamp],depth[stamp];info=infos[0]
            try:
                if (raw.width,raw.height)!=(color.width,color.height) or (raw.width,raw.height)!=(info.width,info.height):
                    raise ValueError('Registered RGB/depth/CameraInfo grid dimensions differ')
                if raw.header.frame_id!=info.header.frame_id:
                    raise ValueError('Depth and CameraInfo optical frames differ')
                tf=buffer.lookup_transform(a.target_frame,raw.header.frame_id,
                    rclpy.time.Time.from_msg(raw.header.stamp))
                z=image_array(raw,depth=True)
                if raw.encoding=='16UC1':
                    if a.depth_scale_m is None:raise ValueError('16UC1 needs explicit verified --depth-scale-m')
                    z*=a.depth_scale_m
                bgr=image_array(color);points,matrix=metric_cloud(z,info,tf)
                measured=None
                if a.joint_topic:
                    if not joints:raise ValueError('No measured JointState received')
                    joint=joints[0];order=[joint.name.index(n) for n in a.joint_names]
                    q=np.asarray(joint.position)[order]
                    if not np.isfinite(q).all():raise ValueError('Nonfinite measured joints')
                    measured=dict(q_rad=q.tolist(),joint_names=a.joint_names,stamp_ns=stamp_ns(joint),
                        image_delta_s=(stamp_ns(joint)-stamp)/1e9,
                        source='received measured JointState; timestamp offset reported, not assumed synchronized')
                if not cv2.imwrite(str(a.output/'camera_rgb.png'),bgr):raise OSError('PNG write failed')
                np.savez_compressed(a.output/'camera_raw.npz',points_world=points,depth_axial_z_m=z)
                report.update(status='REAL_ROS_RGBD_CAPTURED',stamp_ns=stamp,
                    captured_at_client_utc=datetime.now(timezone.utc).isoformat(),
                    rgb_frame=color.header.frame_id,depth_frame=raw.header.frame_id,
                    camera_info_stamp_ns=stamp_ns(info),T_target_camera=matrix.tolist(),
                    tf_lookup_stamp_ns=stamp,depth_encoding=raw.encoding,
                    depth_scale_m=a.depth_scale_m if raw.encoding=='16UC1' else 1.,
                    intrinsics=list(info.k),distortion=list(info.d),distortion_model=info.distortion_model,
                    measured_joints=measured,finite_depth_fraction=float(np.isfinite(points).all(2).mean()),
                    points_world_key_frame=a.target_frame,rgb_file='camera_rgb.png',cloud_file='camera_raw.npz')
                break
            except Exception as exc:
                last_error=f'{type(exc).__name__}: {exc}'
        if report['status']=='NO_OBSERVATION':
            report.update(error=last_error or 'No exact-stamp RGB/depth pair with CameraInfo received',
                          received_rgb=len(rgb),received_depth=len(depth),camera_info_received=bool(infos))
    finally:
        report['capture_wall_s']=time.monotonic()-began
        (a.output/'observation.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(report,indent=2))
    return 0 if report['status']=='REAL_ROS_RGBD_CAPTURED' else 2


if __name__=='__main__':
    raise SystemExit(main())
