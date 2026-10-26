"""Greedy mixed8 -> real cuRoboV2 joint trajectories -> sampled CAD -> Newton FK.

Ideal held-entry initialization and frozen releases; no dynamics or real robot.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

from compare import ROOT, SCENE, UPSTREAM, Box, p
from compact_prior import CompactPrior
from newton_compare import TopWorkspace
from batch_endpoints import solve as solve_endpoints
from online_packing_newton import make_model
from scene_frames import container_pose, rotation, transform_point
from v2_backend import Backend
import newton
import warp as wp
sys.path.insert(0, str(ROOT/"experiments/s20_newton_fullcycle20_20261021"))
from collision import Collision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--catalog", type=Path, help="Prior geometry case supplies simulation arrival sizes only, not targets or paths")
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    folder = Path(__file__).parent/"v2_s20_model_h100"
    began = time.perf_counter()
    cfg = yaml.safe_load(SCENE.read_text())
    hull = p.Hull.from_config(cfg)
    ct = container_pose(cfg)
    cr = rotation(ct)
    catalog = json.loads(args.catalog.read_text())["source_catalog_m"][:args.count] if args.catalog else ([[.55, .4, .25], [.65, .45, .30]]*4)[:args.count]
    manifest = dict(container=hull.as_dict(), steps=[dict(incoming=dict(id=f"B{i+1}", dims=d)) for i, d in enumerate(catalog)])
    wp.set_device("cuda:0")
    model, ids, _, _ = make_model(manifest, UPSTREAM/"policy/vendor/roboticarm", SCENE)
    state, fk = model.state(), model.state()
    tcp = model.body_label.index("S20/suction_contact_frame")
    q0 = model.joint_q.numpy().copy()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    w = SimpleNamespace(model=model, state=state, fk=fk, boxes=list(ids.values()),
        q=q0.copy(), qidx=np.arange(6), tcp=tcp, arm_count=min(ids.values()), held=None,
        grip_offset=np.array([0., 0., .125]), grip_rot=R.from_euler("x", np.pi))
    cad = Collision(w)
    for shape in list(cad.objs):
        body = cad.bodies[shape]
        if body >= 0 and model.body_label[body].endswith("/payload_box"):
            del cad.objs[shape]
            cad.arm.discard(shape)
    domain = json.loads((UPSTREAM/"policy/demo_data/site_four_colors_right160_observe120/domain.json").read_text())["poses"]
    entry_source = np.load(Path(__file__).parent/"compact_contact_motion3_final/trajectory.npz")
    q_entry = entry_source["joint_q"][np.flatnonzero(entry_source["stage"]=="B1_insert")[0]].copy()
    ik = TopWorkspace(UPSTREAM/"policy/vendor/roboticarm", SCENE)
    ik.position_tolerance_m = .01
    bound = json.loads((folder/"reach_bound.json").read_text())
    compact = CompactPrior(cfg, hull, max_box_top_m=1., depth_step_m=.1)
    compact.horizontal_first = True
    compact.row_first = True
    compact.radius = 1.70
    compact.center = state.body_q.numpy()[model.body_label.index("S20/elfin_base"), :3]
    backend = Backend(model, state, folder, out, q_entry)
    bound["center_world_m"] = compact.center.tolist()
    bound["operational_radius_m"] = compact.radius
    bound["operational_radius_source"] = "S20 RL run_live_online_5cm.py -> run_sphere95_layout.py --full-reach --landing-only"
    bound["radius_m_role"] = "Theoretical triangle-inequality outer bound only; NOT the candidate-filter radius"
    frames, joints, stages, placed, frozen, cycles, events = [], [], [], [], {}, [], []
    initial = state.body_q.numpy()
    for body in ids.values():
        initial[body] = [100., 100., 100., 0., 0., 0., 1.]
    state.body_q.assign(initial)
    report = dict(scope=__doc__, source_catalog_m=catalog, scene=str(SCENE), reach=bound,
        max_placed_box_top_m=1., backend="Official cuRoboV2 MotionPlanner, 78fd485fa82d9b9a063fb4985e371814587e666a",
        endpoint_prefilter="Native Newton batched IK only; V2 generates every accepted movement",
        proxy_dimensions=True, release="Ideal nominal frozen release after 5mm pre-release gap; not natural settling",
        validation="V2 sphere world/self plus independent CAD at 20ms; not continuous GSIP/SVSDF proof",
        initialization_s=time.perf_counter()-began, v2_initialization_s=backend.initialization_s,
        cycles=cycles, events=events, placed=0)
    for file in (Path(__file__), Path(__file__).with_name("v2_backend.py"), Path(__file__).with_name("compact_prior.py")):
        shutil.copy2(file, out/("source_"+file.name))
    shutil.copy2(SCENE, out/"scene_tf.yaml")
    shutil.copy2(folder/"robot.yml", out/"robot.yml")

    def save():
        report["wall_s"] = time.perf_counter()-began
        report["placed"] = sum(row["status"] == "kinematic_entry_place_return" for row in cycles)
        (out/"result.json").write_text(json.dumps(report, indent=2))
        if frames:
            np.savez_compressed(out/"trajectory.npz", body_q=np.array(frames),
                joint_q=np.array(joints), stage=np.array(stages), body_labels=np.array(model.body_label))

    def frame(q, stage):
        w.q[:6] = q
        state.joint_q.assign(w.q)
        newton.eval_fk(model, state.joint_q, state.joint_qd, state)
        poses = state.body_q.numpy()
        for label, pose in frozen.items():
            poses[ids[label]] = pose
        if w.held is not None:
            contact = poses[tcp]
            rot = R.from_quat(contact[3:])
            poses[w.held] = np.r_[contact[:3]+rot.apply(w.grip_offset), (rot*w.grip_rot).as_quat()]
        state.body_q.assign(poses)
        frames.append(poses.copy())
        joints.append(np.array(q).copy())
        stages.append(stage)

    def validate(path):
        for index, q in enumerate(path):
            if not cad.validate(q):
                return dict(sample=index, reason="full CAD sampled collision", pairs=cad.last_hits)
        return None

    def obstacles():
        return [(b.id, catalog[int(b.id[1:])-1], frozen[b.id]) for b in placed]

    save()
    for arrival, dims in enumerate(catalog):
        label = f"B{arrival+1}"
        w.held = ids[label]
        w.grip_offset = np.array([0., 0., dims[2]/2])
        cad.arrived = set(range(len(placed)))
        cad.current_source = None
        frame(q_entry, f"{label}_held_entry")
        backend.update_world(obstacles())
        attachment_start = time.perf_counter()
        backend.attach(dims, q_entry)
        attachment_s = time.perf_counter()-attachment_start
        t = time.perf_counter()
        candidates, spatial = compact.candidates(dims, placed)
        spatial_s = time.perf_counter()-t
        t = time.perf_counter()
        endpoints, endpoint_stats = solve_endpoints(ik, candidates, dims, hull, domain, yaw_offsets=(0., np.pi))
        endpoint_s = time.perf_counter()-t
        row = dict(arrival=arrival+1, size=20 if arrival % 2 == 0 else 24,
            status="unresolved", space_planning_s=spatial_s, endpoint_prefilter_s=endpoint_s,
            attachment_s=attachment_s, candidates=len(candidates), endpoint_stats=endpoint_stats,
            trajectory_planning_s=0., cad_validation_s=0., attempts=0)
        selected = None
        for index, (box, answer) in enumerate(zip(candidates, endpoints)):
            if not answer["ok"]:
                continue
            route = p.route_to(box, placed, hull, min_entry_z=.4)
            if route is None:
                continue
            variants = [v for v in answer["variants"] if v["ok"]]
            for variant in variants:
                row["attempts"] += 1
                yaw = ik.box_yaw(dims, box.dims)
                # Plan a single smooth approach from entry; only the final
                # short descent is a separate contact event segment.
                target = np.array(box.pos)+[0., 0., .005]
                position, world_yaw = ik.local_contact_world(target, box.dims, yaw, hull)
                quaternion = ik.tool_orientation(world_yaw+variant["yaw_offset_rad"])
                path, timing = backend.plan(q_entry, position, quaternion)
                row["trajectory_planning_s"] += timing["wall_s"]
                if path is None:
                    events.append(dict(arrival=arrival+1, candidate=index, reason="V2 unresolved", **timing))
                    continue
                started = time.perf_counter()
                error = validate(path)
                row["cad_validation_s"] += time.perf_counter()-started
                if error:
                    events.append(dict(arrival=arrival+1, candidate=index, **error))
                    continue
                selected = box, path, index, variant["yaw_offset_rad"]
                break
            if selected:
                break
        if selected is None:
            cycles.append(row)
            save()
            print(json.dumps(row), flush=True)
            break
        box, path, index, yaw_offset = selected
        for q in path:
            frame(q, f"{label}_insert")
        yaw = ik.box_yaw(dims, box.dims)
        center = np.array(box.pos)+np.array(box.dims)/2+[hull.door_x, hull.inner_y0, hull.floor_z]
        pose = np.r_[transform_point(ct, center), (cr*R.from_euler("z", yaw)).as_quat()]
        frozen[label] = pose
        placed.append(Box(label, box.dims, box.pos))
        w.held = None
        cad.arrived = set(range(len(placed)))
        frame(path[-1], f"{label}_release")
        backend.detach()
        backend.update_world(obstacles())
        empty, timing = backend.plan(path[-1], goal_q=q_entry)
        row["return_planning_s"] = timing["wall_s"]
        if empty is None:
            row["status"] = "return_unresolved"
        else:
            started = time.perf_counter()
            error = validate(empty)
            row["cad_validation_s"] += time.perf_counter()-started
            if error:
                row["status"] = "return_CAD_rejected"
                events.append(dict(arrival=arrival+1, stage="return", **error))
            else:
                for q in empty:
                    frame(q, f"{label}_empty_return")
                row["status"] = "kinematic_entry_place_return"
        row.update(box=asdict(placed[-1]), chosen_candidate=index, tool_yaw_offset_deg=float(np.degrees(yaw_offset)),
                   loaded_duration_s=len(path)*.02, empty_duration_s=len(empty)*.02 if empty is not None else None,
                   loaded_frames=len(path), space_stats=spatial)
        cycles.append(row)
        save()
        print(json.dumps(row), flush=True)
        if row["status"] != "kinematic_entry_place_return":
            break
    save()


if __name__ == "__main__":
    main()
