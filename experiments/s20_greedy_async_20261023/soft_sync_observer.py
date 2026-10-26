"""Read-only RGB-D exact-stamp join + joint interpolation at image acquisition time.

Subscribes to existing ROS streams. Does not open an SDK connection or publish
motion, IO, TF, configuration or rewritten sensor topics.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import time
import numpy as np


def interpolate_joints(samples, stamp):
    rows=sorted(samples,key=lambda r:r[0])
    for left,right in zip(rows,rows[1:]):
        if left[0]<=stamp<=right[0] and right[0]>left[0]:
            alpha=(stamp-left[0])/(right[0]-left[0])
            q0,q1=np.array(left[1]),np.array(right[1])
            delta=(q1-q0+np.pi)%(2*np.pi)-np.pi
            return dict(q_rad=(q0+alpha*delta).tolist(),bracket_s=[left[0],right[0]],
                        bracket_width_s=right[0]-left[0],nearest_skew_s=min(stamp-left[0],right[0]-stamp),
                        joint_names=left[2],method="acquisition-stamp joint interpolation")
    return None


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,JointState,CameraInfo
    from std_msgs.msg import String
    import message_filters
    import tf2_ros
    from rclpy.time import Time
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--duration",type=float,default=8.)
    parser.add_argument("--publish-topic",help="Optional independent String metadata topic; never rewrites existing streams")
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Node("s20_independent_soft_sync_observer")
    publisher=node.create_publisher(String,args.publish_topic,10) if args.publish_topic else None
    joints=deque(maxlen=400);pending=deque(maxlen=12);records=[];counts=dict(rgb=0,depth=0,joint=0,paired=0,interpolated=0)
    info={};snapshot={};pair_rows=[];stream_stats={};tfbuf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(tfbuf,node)
    ns=lambda s:int(s.sec)*10**9+int(s.nanosec)
    def on_joint(msg):
        counts["joint"]+=1
        selected=[i for i,n in enumerate(msg.name) if n.split('/')[-1] in [f"elfin_joint{k}" for k in range(1,7)]]
        selected.sort(key=lambda i:int(msg.name[i][-1]))
        if len(selected)==6:joints.append((ns(msg.header.stamp)/1e9,[msg.position[i] for i in selected],[msg.name[i] for i in selected]))
    sub=node.create_subscription(JointState,"/joint_states",on_joint,qos_profile_sensor_data)
    ci=node.create_subscription(CameraInfo,"/camera/d555/color/camera_info",lambda m:info.update(K=list(m.k),width=m.width,height=m.height),qos_profile_sensor_data)
    def on_stats(name,msg):
        try:stream_stats[name]=json.loads(msg.data)
        except ValueError:stream_stats[name]=dict(raw=msg.data)
    stat_sub=node.create_subscription(String,"/semantic_segmenter/stats_json",lambda m:on_stats('segmenter',m),10)
    detector_sub=node.create_subscription(String,"/luggage_detector/stream_stats_json",lambda m:on_stats('detector',m),10)
    color=message_filters.Subscriber(node,Image,"/camera/d555/color/image_raw",qos_profile=qos_profile_sensor_data)
    depth=message_filters.Subscriber(node,Image,"/camera/d555/aligned_depth_to_color/image_raw",qos_profile=qos_profile_sensor_data)
    color.registerCallback(lambda m:counts.update(rgb=counts["rgb"]+1))
    depth.registerCallback(lambda m:counts.update(depth=counts["depth"]+1))
    join=message_filters.TimeSynchronizer([color,depth],5)
    def on_pair(rgb,d):
        counts["paired"]+=1;pending.append((rgb,d,time.time()))
        snapshot.update(rgb=rgb,depth=d)
        stamp=ns(rgb.header.stamp)/1e9
        pair_rows.append(dict(stamp_s=stamp,age_s=time.time()-stamp,rgb_depth_skew_s=(ns(d.header.stamp)-ns(rgb.header.stamp))/1e9))
    join.registerCallback(on_pair)
    start=time.monotonic()
    while time.monotonic()-start<args.duration:
        rclpy.spin_once(node,timeout_sec=.01)
        for rgb,d,receipt in list(pending):
            stamp=ns(rgb.header.stamp)/1e9;aligned=interpolate_joints(joints,stamp)
            if aligned is None:continue
            counts["interpolated"]+=1
            row=dict(image_stamp_s=stamp,receipt_s=receipt,image_age_s=receipt-stamp,rgb_depth_skew_s=(ns(d.header.stamp)-ns(rgb.header.stamp))/1e9,**aligned)
            try:
                tf=tfbuf.lookup_transform("world",rgb.header.frame_id,Time.from_msg(rgb.header.stamp))
                tr=tf.transform.translation;qr=tf.transform.rotation
                row["T_world_camera_translation"]=[tr.x,tr.y,tr.z];row["T_world_camera_xyzw"]=[qr.x,qr.y,qr.z,qr.w]
            except Exception as exc:row["tf_unavailable"]=str(exc)
            records.append(row);pending.remove((rgb,d,receipt))
            if publisher is not None:
                msg=String();msg.data=json.dumps(dict(**row,camera_info=info,motion_requested=False));publisher.publish(msg)
            snapshot=dict(rgb=rgb,depth=d,row=row)
    if snapshot:
        import cv2
        rgb,d=snapshot["rgb"],snapshot["depth"]
        image=np.frombuffer(rgb.data,dtype=np.uint8).reshape(rgb.height,rgb.step)[:,:rgb.width*3].reshape(rgb.height,rgb.width,3)
        if rgb.encoding=="rgb8":image=image[:,:,::-1]
        cv2.imwrite(str(args.output/"color.png"),image)
        dtype=np.dtype('>u2' if d.is_bigendian else '<u2') if d.encoding in ('16UC1','mono16') else np.dtype('>f4' if d.is_bigendian else '<f4')
        z=np.frombuffer(d.data,dtype=dtype).reshape(d.height,d.step//dtype.itemsize)[:,:d.width]
        np.savez_compressed(args.output/"snapshot.npz",depth=z,K=np.array(info.get("K",[])).reshape(3,3),q=np.array(snapshot.get("row",{}).get("q_rad",[])))
    report=dict(scope=__doc__,motion_requested=False,counts=counts,camera_info=info,
                timing_basis="existing mapped acquisition headers; interpolation is not hardware synchronization",
                image_age_median_ms=float(np.median([r["age_s"] for r in pair_rows])*1000) if pair_rows else None,
                image_age_p95_ms=float(np.percentile([r["age_s"] for r in pair_rows],95)*1000) if pair_rows else None,
                stream_stats=stream_stats,pair_rows=pair_rows,
                joint_nearest_p95_ms=float(np.percentile([r["nearest_skew_s"] for r in records],95)*1000) if records else None,
                joint_bracket_p95_ms=float(np.percentile([r["bracket_width_s"] for r in records],95)*1000) if records else None,
                tf_success=sum("tf_unavailable" not in r for r in records),records=records)
    (args.output/"result.json").write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ("records","pair_rows","stream_stats")}))
    node.destroy_node();rclpy.shutdown()


if __name__=="__main__":main()
