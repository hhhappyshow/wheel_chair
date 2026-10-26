"""Official cuRoboV2 motion generation on the current S20/ULD model."""
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

from compare import ROOT
sys.path.insert(0, str(ROOT/"reproductions/nvidia_curobo_v2_20261023"))
import torch
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState
from curobo._src.geom.types import SceneCfg, Mesh, Cuboid
from curobo._src.cost.tool_pose_criteria import ToolPoseCriteria


class Backend:
    def __init__(self, model, state, model_folder, output, entry_q):
        began = time.perf_counter()
        self.output = output
        self.model = model
        self.base = model.body_label.index("S20/elfin_base")
        base = state.body_q.numpy()[self.base]
        self.base_translation = base[:3]
        self.base_rotation = R.from_quat(base[3:])
        self.meshes = []
        for index, body in enumerate(model.shape_body.numpy()):
            if body >= 0 or not model.shape_flags.numpy()[index] & 2:
                continue
            shape = model.shape_source[index]
            xf = model.shape_transform.numpy()[index]
            scale = model.shape_scale.numpy()[index]
            if shape is not None and hasattr(shape, "_vertices"):
                vertices = np.asarray(shape._vertices)*scale
                faces = np.asarray(shape._indices).ravel()
            else:
                import trimesh
                box = trimesh.creation.box(extents=2*scale)
                vertices, faces = box.vertices, box.faces.ravel()
            vertices = R.from_quat(xf[3:]).apply(vertices)+xf[:3]
            vertices = self.base_rotation.inv().apply(vertices-self.base_translation)
            self.meshes.append(Mesh(name=model.shape_label[index], vertices=vertices.tolist(),
                                    faces=faces.tolist(), pose=[0, 0, 0, 1, 0, 0, 0]))
        robot = yaml.safe_load((model_folder/"robot.yml").read_text())
        robot["robot_cfg"]["kinematics"]["cspace"]["default_joint_position"] = np.asarray(entry_q).tolist()
        trajopt = yaml.safe_load((ROOT/"reproductions/nvidia_curobo_v2_20261023/curobo/content/configs/task/trajopt/lbfgs_bspline_trajopt.yml").read_text())
        trajopt["optimizer"]["fixed_iters"] = False
        self.planner = MotionPlanner(MotionPlannerCfg.create(
            robot=robot,
            trajopt_optimizer_configs=[trajopt],
            scene_model=SceneCfg(mesh=self.meshes), collision_cache={"mesh": len(self.meshes), "cuboid": 12},
            num_ik_seeds=32, num_trajopt_seeds=4, interpolation_dt=.02,
            interpolation_buffer_size=1500, position_tolerance=.005,
            orientation_tolerance=.05, optimizer_collision_activation_distance=0.))
        # CUDA graph replay remains enabled. Only defer expensive PRM warmup
        # until a query actually needs graph seeding.
        self.planner.warmup(enable_graph=False, num_warmup_iterations=1)
        self.planner.update_tool_pose_criteria({"suction_contact_frame": ToolPoseCriteria(
            non_terminal_pose_axes_weight_factor=[0, 0, 0, 1, 1, 0],
            non_terminal_pose_convergence_tolerance=[1e3, .174533],
            terminal_pose_convergence_tolerance=[.005, .05])})
        self.attachments = {}
        self.initialization_s = time.perf_counter()-began
        self.events = []
        print(json.dumps(dict(stage="v2_ready", initialization_s=self.initialization_s,
                              joint_names=self.planner.joint_names)), flush=True)

    def hold_tool_down(self, enabled, freeze_yaw=False, free_translation_axis=None):
        if free_translation_axis is not None:
            criteria=ToolPoseCriteria.linear_motion(axis="xyz"[free_translation_axis])
            criteria.terminal_pose_convergence_tolerance=self.planner.config.device_cfg.to_device([.005,.05])
            self.planner.update_tool_pose_criteria({"suction_contact_frame":criteria})
            return
        self.planner.update_tool_pose_criteria({"suction_contact_frame": ToolPoseCriteria(
            non_terminal_pose_axes_weight_factor=[0,0,0,1,1,int(freeze_yaw)] if enabled else [0]*6,
            non_terminal_pose_convergence_tolerance=[1e3,.174533],
            terminal_pose_convergence_tolerance=[.005,.05],project_distance_to_goal=free_translation_axis is not None)})

    def joints(self, q):
        return JointState.from_position(torch.tensor(np.array(q)[None], device="cuda", dtype=torch.float32),
                                        joint_names=self.planner.joint_names)

    def update_world(self, cargo):
        obstacles = []
        for label, dims, pose in cargo:
            position = self.base_rotation.inv().apply(np.array(pose[:3])-self.base_translation)
            quat = (self.base_rotation.inv()*R.from_quat(pose[3:])).as_quat()
            obstacles.append(Cuboid(name=label, dims=list(dims), pose=[*position, quat[3], *quat[:3]]))
        self.planner.update_world(SceneCfg(mesh=self.meshes, cuboid=obstacles))

    def attach(self, dims, q, grip_offset=None, grip_rotation=None):
        dims = tuple(dims)
        if dims not in self.attachments:
            cuboid = Cuboid(name="held", dims=list(dims), pose=[0, 0, 0, 1, 0, 0, 0])
            spheres = self.planner.attachment_manager.fit_spheres([cuboid], num_spheres=128)
            half = torch.tensor(dims, device="cuda", dtype=torch.float32)/2
            spheres[:,:3] = torch.maximum(torch.minimum(spheres[:,:3],half-1e-5),-half+1e-5)
            spheres[:,3] = torch.minimum(spheres[:,3],(half-spheres[:,:3].abs()).amin(-1))
            spheres[:,3] = torch.where(spheres[:,3]>.001,spheres[:,3]-.001,-100.)
            self.attachments[dims] = spheres
        spheres = self.attachments[dims].clone()
        offset = np.array([0.,0.,dims[2]/2]) if grip_offset is None else np.asarray(grip_offset)
        rotation = R.from_euler("x",np.pi) if grip_rotation is None else grip_rotation
        spheres[:,:3] = torch.tensor(rotation.apply(spheres[:,:3].cpu().numpy())+offset,device="cuda",dtype=torch.float32)
        self.planner.attachment_manager.update(spheres, self.joints(q), link_name="payload_box")

    def detach(self):
        self.planner.attachment_manager.detach("payload_box")

    def pose(self, position, quaternion):
        position = self.base_rotation.inv().apply(np.array(position)-self.base_translation)
        quat = (self.base_rotation.inv()*R.from_quat(quaternion)).as_quat()
        return GoalToolPose(tool_frames=self.planner.tool_frames,
            position=torch.tensor(position.reshape(1, 1, 1, 1, 3), device="cuda", dtype=torch.float32),
            quaternion=torch.tensor(np.r_[quat[3], quat[:3]].reshape(1, 1, 1, 1, 4), device="cuda", dtype=torch.float32))

    def endpoint_seeds(self,q,position,quaternion):
        result=self.planner.ik_solver.solve_pose(self.pose(position,quaternion),self.joints(q),return_seeds=4)
        torch.cuda.synchronize()
        good=result.success.detach().cpu().numpy().reshape(-1)
        solutions=result.solution.detach().cpu().numpy().reshape(-1,6)
        return solutions[good]

    def plan(self, q, position=None, quaternion=None, goal_q=None, seed_q=None, seed_traj=None):
        began = time.perf_counter()
        if goal_q is not None:
            result = self.planner.plan_cspace(self.joints(goal_q), self.joints(q), max_attempts=2)
        elif seed_q is not None:
            seeds=self.joints(seed_q).position[:,None,:].repeat(1,self.planner.trajopt_solver.config.num_seeds,1)
            result=self.planner.trajopt_solver.solve_pose(self.pose(position,quaternion),self.joints(q),
                seed_config=seeds,seed_traj=None if seed_traj is None else torch.tensor(seed_traj[None,None],device="cuda",dtype=torch.float32),
                use_implicit_goal=False,finetune_attempts=1,
                initial_iters=25,time_optimal_iters=25,finetune_iters=25)
            if not bool(result.success.any()):
                result=self.planner.plan_pose(self.pose(position,quaternion),self.joints(q),max_attempts=2)
        else:
            result = self.planner.plan_pose(self.pose(position, quaternion), self.joints(q), max_attempts=2)
        torch.cuda.synchronize()
        elapsed = time.perf_counter()-began
        ok = result is not None and bool(result.success.any())
        event = dict(wall_s=elapsed, success=ok,nearby_ik_seed=seed_q is not None,cartesian_seed=seed_traj is not None)
        if result is not None:
            for field in ("total_time", "solve_time", "status"):
                value = getattr(result, field, None)
                event[field] = str(value)
        self.events.append(event)
        with (self.output/"v2_queries.jsonl").open("a") as stream:
            stream.write(json.dumps(event)+"\n")
        if not ok:
            return None, event
        path = result.get_interpolated_plan().position.detach().cpu().numpy().reshape(-1, 6)
        return path, event
