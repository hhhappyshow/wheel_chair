"""Inspect first-object V2 FK and IK failure terms on the exact current cell."""
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

from compare import ROOT, SCENE, UPSTREAM, p
from online_packing_newton import make_model
from v2_backend import Backend
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo._src.geom.types import SceneCfg
from curobo._src.geom.collision.collision_scene import SceneCollision, SceneCollisionCfg
from curobo._src.geom.collision.buffer_collision import CollisionBuffer
from curobo._src.types.device_cfg import DeviceCfg
import newton
import torch
import warp as wp


def main():
    out = Path(__file__).parent/"v2_first_diagnosis"
    out.mkdir(exist_ok=True)
    folder = Path(__file__).parent/"v2_s20_model_h100"
    cfg = yaml.safe_load(SCENE.read_text())
    hull = p.Hull.from_config(cfg)
    wp.set_device("cuda:0")
    model, _, _, _ = make_model(dict(container=hull.as_dict(), steps=[dict(incoming=dict(id="B1", dims=[.55,.4,.25]))]),
                              UPSTREAM/"policy/vendor/roboticarm", SCENE)
    state = model.state()
    domain = json.loads((UPSTREAM/"policy/demo_data/site_four_colors_right160_observe120/domain.json").read_text())["poses"]
    data=np.load(Path(__file__).parent/"compact_contact_motion3_final/trajectory.npz")
    q = data["joint_q"][np.flatnonzero(data["stage"]=="B1_insert")[-1]].copy()
    state.joint_q.assign(q)
    newton.eval_fk(model,state.joint_q,state.joint_qd,state)
    native = state.body_q.numpy()[model.body_label.index("S20/suction_contact_frame")]
    backend = Backend(model,state,folder,out,q)
    fk = backend.planner.compute_kinematics(backend.joints(q)).tool_poses.get_link_pose("suction_contact_frame")
    position = fk.position.detach().cpu().numpy().reshape(-1,3)[0]
    quat = fk.quaternion.detach().cpu().numpy().reshape(-1,4)[0]
    actual = backend.base_translation+backend.base_rotation.apply(position)
    report = dict(native_tcp=native.tolist(),v2_tcp_world_m=actual.tolist(),
                  fk_position_error_m=float(np.linalg.norm(actual-native[:3])),
                  fk_rotation_error_rad=float(((backend.base_rotation*R.from_quat(np.r_[quat[1:],quat[0]])).inv()*R.from_quat(native[3:])).magnitude()),cases=[])
    goal = backend.pose(native[:3],native[3:])
    backend.attach([.55,.4,.25],q)
    spheres = backend.planner.compute_kinematics(backend.joints(q)).robot_spheres
    params = backend.planner.trajopt_solver.core.attachment_manager.kinematics_params
    names = {v:k for k,v in params.link_name_to_idx_map.items()}
    mapping = params.link_sphere_idx_map.cpu().numpy()
    pairs=backend.planner.kinematics.get_self_collision_config().collision_pairs.cpu().numpy()
    raw=spheres.detach().cpu().numpy().reshape(-1,4)
    overlaps=[]
    for i,j in pairs:
        depth=float(raw[i,3]+raw[j,3]-np.linalg.norm(raw[i,:3]-raw[j,:3]))
        if depth>0:overlaps.append(dict(links=[names[int(mapping[i])],names[int(mapping[j])]],depth_m=depth))
    report["attached_self_overlaps"]=overlaps
    print(json.dumps(dict(attached_self_overlaps=overlaps[:20])),flush=True)
    report["world_collision_at_start"] = []
    for mesh in backend.meshes:
        scene = SceneCollision.from_config(SceneCollisionCfg(device_cfg=DeviceCfg(),scene_model=SceneCfg(mesh=[mesh])))
        buffer = CollisionBuffer.from_shape(spheres.shape,DeviceCfg())
        distance = scene.get_sphere_distance_raw(query_spheres=spheres,collision_buffer=buffer,
            weight=torch.ones(1,device="cuda"),activation_distance=torch.zeros(1,device="cuda")).detach().cpu().numpy().ravel()
        indices = np.flatnonzero(distance>0)
        hits = dict(mesh=mesh.name,links=sorted({names[int(mapping[i])] for i in indices}),maximum_cost=float(distance.max()),spheres=len(indices))
        report["world_collision_at_start"].append(hits)
        print(json.dumps(hits),flush=True)
    for name, planner in [("full_world_self",backend.planner),
        ("empty_world_self",MotionPlanner(MotionPlannerCfg.create(robot=str((folder/"robot.yml").resolve()),self_collision_check=True))),
        ("empty_world_no_self_diagnostic",MotionPlanner(MotionPlannerCfg.create(robot=str((folder/"robot.yml").resolve()),self_collision_check=False)))]:
        result = planner.ik_solver.solve_pose(goal,current_state=backend.joints(q),return_seeds=4)
        torch.cuda.synchronize()
        row = dict(case=name,success=result.success.cpu().tolist(),position_error=result.position_error.cpu().tolist(),
                   rotation_error=result.rotation_error.cpu().tolist(),solution=result.solution.cpu().tolist())
        report["cases"].append(row)
        print(json.dumps(row),flush=True)
    (out/"result.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)


if __name__=="__main__":main()
