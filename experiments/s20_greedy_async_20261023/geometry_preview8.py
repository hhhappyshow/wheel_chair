"""Online mixed-box spatial plan and native Newton kinematic storyboard.

Current box plus committed occupancy only. No arm, IK, dynamics or grasp model.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from compare import Box, SCENE, UPSTREAM, p
from compact_prior import CompactPrior


PALETTE = [(.17, .65, .67), (.90, .59, .22), (.66, .40, .70),
           (.40, .70, .35), (.85, .35, .40), (.35, .55, .85),
           (.80, .70, .30), (.65, .65, .65)]
CATALOG = [[.55, .40, .25], [.65, .45, .30]] * 4


def plan(output, catalog=CATALOG, constrained=False, seed=None, row_first=False, contact_first=False):
    cfg = yaml.safe_load(SCENE.read_text())
    hull = p.Hull.from_config(cfg)
    prior = CompactPrior(cfg, hull, use_endpoint_sphere=constrained,
                         max_box_top_m=1. if constrained else None)
    prior.horizontal_first = True
    prior.row_first = row_first
    prior.contact_first = contact_first
    placed, steps = [], []
    for index, dims in enumerate(catalog):
        began = time.perf_counter()
        candidates, timing = prior.candidates(dims, placed)
        ranked = time.perf_counter()
        selected = None
        for checked, candidate in enumerate(candidates, 1):
            route = p.route_to(candidate, placed, hull)
            if route is None:
                continue
            # Author routes use a 0.1mm coordinate grid. Check exactly the
            # coordinates that will be rendered, not a nearby unrounded target.
            candidate = Box(f"B{index+1}", candidate.dims, tuple(route[-1]))
            if prior.valid(candidate, placed) and p.path_clear(candidate.dims, route, placed, hull):
                selected = candidate, route
                break
        finished = time.perf_counter()
        row = dict(id=f"B{index+1}", size_label="20in +/-10pct" if constrained else "20in" if index % 2 == 0 else "24in",
                   source_dimensions_m=dims, candidates=len(candidates),
                   candidate_generation_and_ranking_s=ranked-began,
                   route_search_s=finished-ranked, planning_s=finished-began,
                   route_checks=checked if candidates else 0,
                   occupied_before=[asdict(b) for b in placed],
                   scene_revision=len(placed), future_arrivals_used=False)
        if selected is None:
            row.update(status="unresolved", reason="No candidate with a checked translational cargo route")
            steps.append(row)
            print(json.dumps(row), flush=True)
            break
        box, route = selected
        neighbor, walls = prior.contacts(box, placed)
        row.update(status="geometric_route_accepted", box=asdict(box), route=route,
                   yaw_deg=90 if box.dims[0] < box.dims[1] else 0,
                   adjacent_contact_area_m2=neighbor, wall_contact_area_m2=walls,
                   route_length_m=p.route_length(route))
        placed.append(box)
        steps.append(row)
        print(json.dumps({k: row[k] for k in ("id", "box", "planning_s", "route_checks")}), flush=True)
    report = dict(scope=__doc__, source_catalog_m=catalog, steps=steps,
                  accepted=len(placed), requested=len(catalog), hull=hull.as_dict(),
                  scene_config=str(SCENE),
                  dimensions_source="Ideal proxy geometry, not camera measurement in this preview",
                  support="Reviewed 5cm edge/height allowance with supported center; no release physics",
                  reach_prior="Base-centered 1.70m final top-center sphere; not an IK certificate" if constrained else "None: spatial layout preview before robot reach/IK consideration",
                  reach_center_world_m=prior.center.tolist(), reach_radius_m=prior.radius if constrained else None,
                  max_box_top_above_floor_m=prior.max_box_top_m,
                  randomization=dict(seed=seed, nominal_m=[.55,.40,.25], independent_axis_uniform=[.9,1.1], dimensions_grid_m=.001) if constrained else None,
                  route_validation="Fixed-yaw axis-aligned swept cargo AABB against hull and committed cargo",
                  ranking="Connected face-contact cluster, same rear row, low height, minimum envelope growth, shared area; yaw tie-breaker" if contact_first else "Rear frontier then low height band then right-to-left; orientation is a local tie-breaker" if row_first else "Horizontal first; right/rear first, then adjacent compact cluster, left/up before outward",
                  row_first=row_first,
                  contact_first=contact_first,
                  robot_execution=False, physics_execution=False)
    (output/"result.json").write_text(json.dumps(report, indent=2))
    shutil.copy2(SCENE, output/"scene_tf.yaml")
    for file in (Path(__file__), Path(__file__).with_name("compact_prior.py")):
        shutil.copy2(file, output/("source_"+file.name))
    return report


def render(output, report):
    import newton
    import newton.viewer
    import trimesh
    import warp as wp
    from PIL import Image
    from online_packing_newton import make_model

    wp.set_device("cuda:0")
    hull = p.Hull.from_dict(report["hull"])
    sample = dict(container=report["hull"], steps=[dict(incoming=dict(
        id=f"B{i+1}", dims=d, color_rgb=PALETTE[i % len(PALETTE)])) for i, d in enumerate(report["source_catalog_m"])])
    model, bodies, transform, _ = make_model(
        sample, UPSTREAM/"policy/vendor/roboticarm", output/"scene_tf.yaml", show_robot=False)
    flags = model.shape_flags.numpy()
    shape_bodies = model.shape_body.numpy()
    for shape, body in enumerate(shape_bodies):
        if body < 0 or model.body_label[body] not in bodies:
            flags[shape] &= ~1
    model.shape_flags.assign(flags)
    rotation = Rotation.from_euler("xyz", transform["rotation_rpy"])
    translation = np.array(transform["translation"])
    state = model.state()
    initial = state.body_q.numpy().copy()
    for body in bodies.values():
        initial[body, :3] = [100., 100., -100.]
        initial[body, 3:] = [0., 0., 0., 1.]

    fps = 20
    frames, stages, active = [], [], []
    committed = {}

    def append(box_id, point, box, stage):
        positions = initial.copy()
        visible = dict(committed)
        visible[box_id] = point, box
        for label, (position, placed) in visible.items():
            center = np.array(position)+np.array(placed["dims"])/2
            local = center+[hull.door_x, hull.inner_y0, hull.floor_z]
            positions[bodies[label], :3] = translation+rotation.apply(local)
            yaw = -np.pi/2 if placed["dims"][0] < placed["dims"][1] else 0.
            positions[bodies[label], 3:] = (rotation*Rotation.from_euler("z", yaw)).as_quat()
        frames.append(positions)
        stages.append(stage)
        active.append(box_id)

    for row in report["steps"]:
        if row["status"] != "geometric_route_accepted":
            continue
        route, box, label = np.array(row["route"]), row["box"], row["id"]
        for _ in range(int(.4*fps)):
            append(label, route[0], box, "outside")
        for start, end in zip(route, route[1:]):
            duration = max(.6, np.linalg.norm(end-start)/.45)
            for alpha in np.linspace(0., 1., int(np.ceil(duration*fps))+1)[1:]:
                # Display interpolation only, not a controller reference.
                alpha = alpha*alpha*(3.-2.*alpha)
                append(label, start+(end-start)*alpha, box, "translation")
        committed[label] = route[-1], box
        for _ in range(int(.6*fps)):
            append(label, route[-1], box, "placed")
    for _ in range(2*fps):
        frames.append(frames[-1].copy())
        stages.append("final_layout")
        active.append("")
    frames = np.array(frames)
    np.savez_compressed(output/"geometry_frames.npz", body_q=frames,
                        body_labels=np.array(model.body_label), stage=stages, active=active, fps=fps)

    viewer = newton.viewer.ViewerGL(width=640, height=480, headless=True)
    viewer.set_model(model)
    viewer.show_ui = False
    shell = model.shape_label.index("container_collision_full")
    source = model.shape_source[shell]
    mesh = trimesh.Trimesh(np.asarray(source._vertices)*model.shape_scale.numpy()[shell],
                           np.asarray(source._indices).reshape(-1, 3), process=True)
    edges = mesh.face_adjacency_edges[mesh.face_adjacency_angles > .2]
    xf = model.shape_transform.numpy()[shell]
    vertices = Rotation.from_quat(xf[3:]).apply(mesh.vertices)+xf[:3]
    starts = wp.array(vertices[edges[:, 0]].astype(np.float32), dtype=wp.vec3)
    ends = wp.array(vertices[edges[:, 1]].astype(np.float32), dtype=wp.vec3)
    def world(point):
        return translation+rotation.apply(np.array(point)+[hull.door_x, hull.inner_y0, hull.floor_z])

    target = np.array([.4, hull.width/2, .6])
    views = [world(target+[-4.3, 0., .45]),
             world(target+[0., .08, 4.5]),
             world(target+[0., -4.5, .05])]
    world_target = world(target)
    video = output/("random20_pm10_newton_spatial_three_views.mp4" if report.get("randomization") else "mixed8_newton_spatial_three_views.mp4")
    encoder = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
        "-pix_fmt", "rgb24", "-s", "1920x480", "-r", str(fps), "-i", "-",
        "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(video)], stdin=subprocess.PIPE)
    for index, positions in enumerate(frames):
        state.body_q.assign(positions)
        panels = []
        for view_index, pos in enumerate(views):
            delta = world_target-pos
            # Use X as the top camera's up axis: world-Z up is singular
            # directly overhead and makes the image roll with the small TF tilt.
            viewer.camera.up_axis = 0 if view_index == 1 else 2
            if view_index == 1:
                pitch = np.degrees(np.arctan2(delta[0], np.linalg.norm(delta[1:])))
                yaw = np.degrees(np.arctan2(delta[2], delta[1]))
            else:
                pitch = np.degrees(np.arctan2(delta[2], np.linalg.norm(delta[:2])))
                yaw = np.degrees(np.arctan2(delta[1], delta[0]))
            viewer.set_camera(pos=wp.vec3(*pos), pitch=pitch, yaw=yaw)
            for _ in range(2):
                viewer.begin_frame(index/fps)
                viewer.log_state(state)
                viewer.log_lines("cad_shell", starts, ends, (.45, .65, .75))
                viewer.end_frame()
            panel = viewer.get_frame().numpy()
            panels.append(np.rot90(panel, 2) if view_index == 1 else panel)
        combined = np.concatenate(panels, axis=1)
        encoder.stdin.write(combined.tobytes())
        if stages[index] == "placed" and (index == len(frames)-1 or active[index+1] != active[index]):
            Image.fromarray(combined).save(output/f"{active[index]}_placed.png")
        if index == len(frames)-1:
            Image.fromarray(combined).save(output/"final_three_views.png")
        if index % 200 == 0:
            print(json.dumps(dict(rendered=index, frames=len(frames))), flush=True)
    encoder.stdin.close()
    code = encoder.wait()
    viewer.close()
    if code:
        raise RuntimeError("Video encoder failed")
    (output/"render.json").write_text(json.dumps(dict(
        frames=len(frames), fps=fps, duration_s=len(frames)/fps,
        source="Native Newton CAD rendering of saved kinematic box translations",
        dynamics=False, robot=False, speed_is_presentation_only=True,
        views="Container-aligned front/top/side; CAD feature edges with visual occluder hidden"), indent=2))
    shutil.copy2(Path(__file__), output/("source_"+Path(__file__).name))
    print(video, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--random20", action="store_true", help="20in proxy, independent +/-10pct axes; 1.70m endpoint sphere and 1m stack height")
    parser.add_argument("--seed", type=int, default=73)
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--row-first", action="store_true", help="Fill rear-row low space with either yaw before rising; horizontal is a tie-breaker")
    parser.add_argument("--contact-first",action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/"result.json"
    catalog = np.round(np.array([.55,.40,.25])*np.random.default_rng(args.seed).uniform(.9,1.1,(args.count,3)),3).tolist() if args.random20 else CATALOG
    report = json.loads(report_path.read_text()) if report_path.exists() else plan(args.output, catalog, args.random20, args.seed, args.row_first,args.contact_first)
    if args.render:
        render(args.output, report)


if __name__ == "__main__":
    main()
