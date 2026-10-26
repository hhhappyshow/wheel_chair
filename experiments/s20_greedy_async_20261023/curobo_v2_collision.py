"""Actual author cuRoboV2 CUDA/Warp batched world collision on exported S20 data."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"reproductions/nvidia_curobo_v2_20261023"))
import torch
from curobo._src.types.device_cfg import DeviceCfg
from curobo._src.geom.types import Cuboid, Mesh, SceneCfg
from curobo._src.geom.collision.collision_scene import SceneCollision,SceneCollisionCfg
from curobo._src.geom.collision.buffer_collision import CollisionBuffer


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-folder",type=Path,required=True)
    parser.add_argument("--sweep",action="store_true")
    args=parser.parse_args()
    meta=json.loads((args.run_folder/"gpu_scene.json").read_text());data=np.load(args.run_folder/"gpu_spheres.npz")
    cfg=DeviceCfg();results=[]
    for batch in meta["batches"]:
        begin=time.perf_counter()
        scene_cfg=SceneCfg(mesh=[Mesh(**row) for row in meta["scene_mesh"]],cuboid=[Cuboid(**row) for row in batch["previous_boxes"]])
        scene=SceneCollision.from_config(SceneCollisionCfg(device_cfg=cfg,scene_model=scene_cfg,max_distance=1.))
        spheres=torch.tensor(data[f"B{batch['arrival']}"],device="cuda").unsqueeze(0)
        buffer=CollisionBuffer.from_shape(spheres.shape,cfg)
        weight=torch.ones(1,device="cuda");activation=torch.zeros(1,device="cuda")
        dt=torch.tensor([.02],device="cuda")
        torch.cuda.synchronize();setup=time.perf_counter()-begin
        def query():
            if args.sweep:
                return scene.get_swept_sphere_distance_raw(query_spheres=spheres,collision_buffer=buffer,weight=weight,activation_distance=activation,trajectory_dt=dt)
            return scene.get_sphere_distance_raw(query_spheres=spheres,collision_buffer=buffer,weight=weight,activation_distance=activation)
        begin=time.perf_counter();cost=query();torch.cuda.synchronize();first=time.perf_counter()-begin
        samples=[]
        for _ in range(20):
            begin=time.perf_counter();cost=query();torch.cuda.synchronize();samples.append(time.perf_counter()-begin)
        frame_collision=(cost>0).any(dim=-1).cpu().numpy().ravel()
        spheres.requires_grad_(True);begin=time.perf_counter();query().sum().backward();torch.cuda.synchronize();gradient_s=time.perf_counter()-begin
        results.append(dict(arrival=batch["arrival"],frames=spheres.shape[1],spheres_per_frame=spheres.shape[2],setup_s=setup,
                            first_query_s=first,batch_median_s=float(np.median(samples)),batch_p95_s=float(np.percentile(samples,95)),
                            flagged_frames=int(frame_collision.sum()),flagged_indices=np.flatnonzero(frame_collision).tolist(),
                            gradient_query_backward_s=gradient_s,nonzero_position_gradients=int((spheres.grad[...,:3].norm(dim=-1)>0).sum().item())))
    report=dict(scope=__doc__,repository="https://github.com/NVlabs/curobo",commit="78fd485fa82d9b9a063fb4985e371814587e666a",
                device=torch.cuda.get_device_name(),torch=torch.__version__,results=results,
                acceptance_backend_changed=False,self_collision_included=False,representation=meta["coverage"],
                query_mode="author swept sphere, SWEEP_STEPS=3; NOT GSIP continuous proof" if args.sweep else "discrete batched spheres",
                interpretation="GPU collision work is batched; FK/sphere generation, IK and complete planning are not timed as this kernel query")
    (args.run_folder/("curobo_v2_sweep.json" if args.sweep else "curobo_v2_collision.json")).write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=="__main__":main()
