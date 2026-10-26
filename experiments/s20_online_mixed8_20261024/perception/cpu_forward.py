"""Actual CPU YOLO forwards over saved RGB-D, with no geometric mask fallback."""
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parent
os.environ['CUDA_VISIBLE_DEVICES'] = ''
os.environ['YOLO_CONFIG_DIR'] = str(OUT/'cpu_forward/yolo_config')
os.environ['MPLCONFIGDIR'] = str(OUT/'cpu_forward/mpl_config')
os.environ['XDG_CACHE_HOME'] = str(OUT/'cpu_forward/cache')
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from yolo_sim_perception import infer, write_json
from grasp_interface import EnvelopeGraspPlanner


def main():
    torch.set_num_threads(2)
    root = OUT.parents[2]
    parent = root/'experiments/s20_greedy_async_20261023'
    weights = Path('/home/ubuntu/yolo11n-seg.pt')
    model = YOLO(str(weights))
    planner = EnvelopeGraspPlanner(parent/'v2_s20_model_h100/s20_current.urdf')
    rows = []
    for label in ('B1','B2','B3','B4'):
        folder = parent/'mixed8_horizontal_observed_attempt2/perception'/label
        output = OUT/'cpu_forward'/label
        output.mkdir(parents=True, exist_ok=True)
        rgb = cv2.imread(str(folder/'camera_rgb.png'))
        with np.load(folder/'camera_observation.npz') as saved:
            cloud = saved['points_world']
        start = time.perf_counter()
        record, mask, fitted = infer(model, rgb, cloud, confidence=.25, device='cpu')
        record.update(wall_s=time.perf_counter()-start, input_rgb=str(folder/'camera_rgb.png'),
                      input_npz=str(folder/'camera_observation.npz'), npz_keys_read=['points_world'])
        cv2.imwrite(str(output/'yolo_overlay.png'), infer.last_overlay)
        if mask is not None:
            cv2.imwrite(str(output/'yolo_selection.png'), mask.astype('uint8')*255)
        if fitted:
            write_json(output/'grasp.json', planner.plan(fitted))
        write_json(output/'result.json', record)
        rows.append(dict(box=label,status=record['status'], luggage_detections=record['luggage_detections'],
                         raw_detections=record['raw_detections'],speed_ms=record['speed_ms'],
                         wall_s=record['wall_s'], result=str(output/'result.json')))
    # Exercise the exact main-loop API separately on the EXISTING GEOMETRIC
    # observed-box model, explicitly not on a fictitious YOLO detection.
    observed_path = parent/'mixed8_horizontal_observed_attempt2/perception/B1/observed_box.json'
    observed = json.loads(observed_path.read_text())
    example = planner.plan(observed, container_center_world_m=[-1.6588,-.0437,1.2708])
    example['input'] = str(observed_path)
    example['input_is_yolo'] = False
    write_json(OUT/'interface_centered_example.json', example)
    versions = {p:importlib.metadata.version(p) for p in ('torch','torchvision','ultralytics','ultralytics-thop','psutil')}
    summary = dict(python=sys.executable, versions=versions, weights=str(weights),
       dependencies_installed_only_to=str(OUT/'pydeps'), original_venv_modified=False,
       device='cpu', torch_threads=torch.get_num_threads(), OMP_NUM_THREADS=os.environ.get('OMP_NUM_THREADS'),
       OPENBLAS_NUM_THREADS=os.environ.get('OPENBLAS_NUM_THREADS'), gpu_render_started=False,
       conf=.25, gt_mask_used=False, saved_geometric_mask_used=False, results=rows,
       interpretation='COCO suitcase recognition on plain cuboids; zero detection remains a real negative result',
       interface_example_source='separate saved geometric model, not YOLO; pose and estimated grasp binding only')
    write_json(OUT/'cpu_forward_summary.json', summary)
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
