"""Convert a real paired D555 capture through current site FK into observed_box.

Uses existing YOLO/top-plane/grasp modules. No simulator, SDK or actuation.
The recorded nearest receipt-time joint is a soft synchronization measurement,
not a claim of calibrated acquisition timing or suction success.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

HERE=Path(__file__).resolve().parent
EXPERIMENT=HERE.parent
_models={}


def load_model(weights):
    sys.path.insert(0,str(EXPERIMENT/'perception'))
    from ultralytics import YOLO
    key=str(Path(weights).resolve())
    if key not in _models:_models[key]=YOLO(key)
    return _models[key]


def driver_transform(rows,parent,child):
    frames={parent:np.eye(4)}
    while child not in frames:
        count=len(frames)
        for row in rows:
            if row['parent'] not in frames or row['child'] in frames:continue
            t=np.eye(4);t[:3,:3]=Rotation.from_quat(row['quaternion_xyzw']).as_matrix()
            t[:3,3]=row['translation_m'];frames[row['child']]=frames[row['parent']]@t
        if len(frames)==count:raise ValueError(f'Missing driver optical TF: {parent} -> {child}')
    return frames[child]


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--site-urdf',type=Path,required=True)
    p.add_argument('--weights',type=Path,required=True)
    p.add_argument('--yolo-confidence',type=float,default=.15)
    p.add_argument('--perception-target',choices=('luggage','any'),default='luggage')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);a.output.mkdir(parents=True,exist_ok=False)
    began=time.monotonic()
    record=json.loads((a.capture/'result.json').read_text());pair=record['paired']
    unit=record['ros_depth_unit_m'];info=record['info']
    if info['distortion_model'] not in ('plumb_bob','rational_polynomial'):
        raise ValueError('Unsupported recorded distortion model: '+info['distortion_model'])
    module_path=EXPERIMENT/'pose_readback/measured_pose.py'
    spec=importlib.util.spec_from_file_location('site_fk',module_path)
    fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)
    cfg=yaml.safe_load(a.scene.read_text());xml=ET.parse(a.site_urdf).getroot()
    q=np.radians(pair['nearest_received_joint_sample']['q_deg'])
    arm=fk.forward(xml,q);ped=cfg['pedestal']
    base=fk.tf(np.array(ped['translation'])+[0,0,ped['size'][2]],
        [0,0,ped['rotation_rpy'][2]+cfg['robot']['rotation_rpy'][2]])
    driver=driver_transform(record['driver_static_transforms'],'d555_check_link',pair['depth_frame'])
    world_camera=base@arm['camera_link'];world_optical=world_camera@driver
    with np.load(a.capture/'depth_paired_raw.npz') as saved:z=saved['depth_raw_units']*unit
    rgb=cv2.imread(str(a.capture/'rgb_paired.png'))
    v,u=np.indices(z.shape);k=np.array(info['k']).reshape(3,3)
    xy=cv2.undistortPoints(np.column_stack((u.ravel(),v.ravel())).astype(float)[:,None,:],
        k,np.array(info['d'])).reshape(*z.shape,2)
    cloud=np.dstack((xy*z[:,:,None],z))@world_optical[:3,:3].T+world_optical[:3,3]
    cloud[~np.isfinite(z)|(z<=0)]=np.nan
    np.savez_compressed(a.output/'camera_raw.npz',points_world=cloud,depth_axial_z_m=z)
    cv2.imwrite(str(a.output/'camera_rgb.png'),rgb)
    sys.path.insert(0,str(EXPERIMENT/'perception'))
    from yolo_sim_perception import infer,write_json
    prediction,mask,fitted=infer(load_model(a.weights),rgb,cloud,
        confidence=a.yolo_confidence,device='cpu',class_agnostic=a.perception_target=='any',
        require_parallel_top=False)
    prediction['detection_confidence_threshold']=a.yolo_confidence
    cv2.imwrite(str(a.output/'yolo_overlay.png'),infer.last_overlay)
    if mask is not None:cv2.imwrite(str(a.output/'yolo_selection.png'),mask.astype('uint8')*255)
    if fitted:write_json(a.output/'observed_box.json',fitted)
    prediction.update(scope=__doc__,capture=str(a.capture),scene=str(a.scene),site_urdf=str(a.site_urdf),
        T_world_camera=world_camera.tolist(),T_camera_optical=driver.tolist(),T_world_optical=world_optical.tolist(),
        depth_unit_m=unit,depth_unit_basis=record['depth_unit_basis'],
        q_rad=q.tolist(),pair_stamp_ns=pair['stamp_ns'],receipt_joint_delta_s=pair['receipt_joint_delta_s'],
        synchronization=pair['synchronization'],hardware_motion=False,io_written=False,
        wall_s=time.monotonic()-began)
    write_json(a.output/'observation.json',prediction)
    print(json.dumps(dict(output=str(a.output),status=prediction.get('status'),
        dimensions_m=fitted.get('dimensions_m') if fitted else None,wall_s=prediction['wall_s'])))


if __name__=='__main__':main()
