"""Prepare on CPU by default. --render explicitly enables native Newton/CUDA."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PALETTE = [(0.17, .65, .67), (.90, .59, .22), (.66, .40, .70), (.40, .70, .35),
           (.85, .35, .40), (.35, .55, .85), (.80, .70, .30), (.65, .65, .65)]


def boxes_by_id(boxes):
    return {b["id"]: deepcopy(b) for b in boxes}


def blend(a, b, fraction):
    fraction = min(1., max(0., fraction))
    smooth = fraction*fraction*(3-2*fraction)
    return [x+(y-x)*smooth for x, y in zip(a, b)]


def route_durations(step):
    route = step["current_route"] or []
    return [max(.6, math.dist(a, b)/.45) for a, b in zip(route, route[1:])]


def route_position(step, seconds):
    route = step["current_route"]
    for a, b, duration in zip(route, route[1:], route_durations(step)):
        if seconds <= duration:
            return blend(a, b, seconds/duration)
        seconds -= duration
    return list(route[-1])


def visible(step, phase, elapsed, duration):
    boxes = boxes_by_id(step["occupied_before"] if phase in ("before", "movement")
                        else step["occupied_after_push"])
    movement = step["movement"]
    if phase == "movement" and movement:
        moving = deepcopy(movement["before"])
        moving["pos"] = blend(movement["before"]["pos"], movement["after"]["pos"], elapsed/duration)
        boxes[moving["id"]] = moving
    current = step["current_choice"]
    if current and phase in ("outside", "insertion", "commit", "final"):
        current = deepcopy(current)
        if phase == "outside":
            current["pos"] = list(step["current_route"][0])
        elif phase == "insertion":
            current["pos"] = route_position(step, elapsed)
        boxes[current["id"]] = current
    return boxes


def prepare(report, fps):
    runs = {r["mode"]: r for r in report["runs"]}
    modes = ("no_push", "push")
    if report["version"] != 2 or any(len(runs[m]["steps"]) != 8 for m in modes):
        raise ValueError("Expected the saved v2 eight-arrival comparison")
    phases = []; prior = {m: {} for m in modes}; total = 0
    for index in range(8):
        steps = [runs[m]["steps"][index] for m in modes]
        if steps[0]["incoming"] != steps[1]["incoming"]:
            raise ValueError("Comparison arrivals differ")
        for mode, step in zip(modes, steps):
            before = boxes_by_id(step["occupied_before"])
            if before != prior[mode]:
                raise ValueError(f"Discontinuous saved prefix: {mode}/{index}")
            expected = deepcopy(before)
            if step["movement"]:
                move = step["movement"]
                if expected[move["box_id"]] != move["before"]:
                    raise ValueError("Movement start differs from saved prefix")
                expected[move["box_id"]] = deepcopy(move["after"])
            if expected != boxes_by_id(step["occupied_after_push"]):
                raise ValueError("Unexpected frozen-box motion")
            if step["current_choice"]:
                if math.dist(step["current_route"][-1], step["current_choice"]["pos"]) > 1e-8:
                    raise ValueError("Insertion does not end at commit")
                expected[step["current_choice"]["id"]] = deepcopy(step["current_choice"])
            prior[mode] = expected
        durations = [("before", .4), ("movement", .8 if any(s["movement"] for s in steps) else .25),
                     ("outside", .4), ("insertion", max(.1, *(sum(route_durations(s)) for s in steps))),
                     ("commit", .6)]
        for phase, duration in durations:
            frames = max(1, math.ceil(duration*fps))
            phases.append(dict(arrival=index+1, phase=phase, first_frame=total, frames=frames, duration_s=frames/fps))
            total += frames
    phases.append(dict(arrival=8, phase="final", first_frame=total, frames=2*fps, duration_s=2.))
    total += 2*fps
    return dict(fps=fps, frames=total, duration_s=total/fps, phases=phases)


def render(report, plan, output, device, width, height):
    # These imports and all graphics/GPU calls are unreachable in prepare-only mode.
    sys.path.insert(0, str(ROOT/"experiments/s20_greedy_async_20261023"))
    from geometry_preview8 import UPSTREAM, p
    from online_packing_newton import make_model
    import newton.viewer
    import numpy as np
    from PIL import Image
    from scipy.spatial.transform import Rotation
    import trimesh
    import warp as wp
    import yaml

    video = output/"compare_perspective.mp4"
    if video.exists():
        raise FileExistsError(f"Keep previous video; choose a new --output: {video}")
    wp.set_device(device)
    hull = p.Hull.from_dict(report["hull"])
    scene_path = output/"scene_tf.yaml"
    scene_path.write_text(yaml.safe_dump(report["scene_config_snapshot"], sort_keys=False))
    modes = ("no_push", "push")
    runs = {r["mode"]: r for r in report["runs"]}
    arrivals = runs["no_push"]["steps"]
    sample = dict(container=report["hull"], steps=[dict(incoming=dict(
        id=f"B{i+1}", dims=s["incoming"]["dimensions_m"], color_rgb=PALETTE[i])) for i, s in enumerate(arrivals)])
    model, bodies, transform, _ = make_model(sample, UPSTREAM/"policy/vendor/roboticarm", scene_path, show_robot=False)
    flags = model.shape_flags.numpy()
    for shape, body in enumerate(model.shape_body.numpy()):
        if body < 0 or model.body_label[body] not in bodies:
            flags[shape] &= ~1
    model.shape_flags.assign(flags)
    rotation = Rotation.from_euler("xyz", transform["rotation_rpy"])
    translation = np.array(transform["translation"])
    state = model.state(); hidden = state.body_q.numpy().copy()
    for body in bodies.values():
        hidden[body, :3] = [100., 100., -100.]
        hidden[body, 3:] = [0., 0., 0., 1.]
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

    target_local = np.array([.25, hull.width/2, .60])
    target = world(target_local); camera = world(target_local+[-3.6, -3.4, 2.4])
    delta = target-camera
    viewer = newton.viewer.ViewerGL(width=width, height=height, headless=True)
    viewer.set_model(model); viewer.show_ui = False; viewer.camera.up_axis = 2
    viewer.set_camera(pos=wp.vec3(*camera),
                      pitch=float(np.degrees(np.arctan2(delta[2], np.linalg.norm(delta[:2])))),
                      yaw=float(np.degrees(np.arctan2(delta[1], delta[0]))))
    encoder = subprocess.Popen(["ffmpeg", "-n", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{2*width}x{height}", "-r", str(plan["fps"]), "-i", "-", "-c:v", "libx264", "-preset", "fast",
        "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video)], stdin=subprocess.PIPE)
    rendered = 0
    try:
        for phase in plan["phases"]:
            for index in range(phase["frames"]):
                panels = []
                for mode in modes:
                    step = runs[mode]["steps"][phase["arrival"]-1]
                    layout = visible(step, phase["phase"], (index+1)/plan["fps"], phase["duration_s"])
                    positions = hidden.copy()
                    for label, box in layout.items():
                        positions[bodies[label], :3] = world(np.array(box["pos"])+np.array(box["dims"])/2)
                        source_dims = arrivals[int(label[1:])-1]["incoming"]["dimensions_m"]
                        yaw = 0. if np.allclose(source_dims, box["dims"], atol=1e-9, rtol=0) else -np.pi/2
                        positions[bodies[label], 3:] = (rotation*Rotation.from_euler("z", yaw)).as_quat()
                    state.body_q.assign(positions)
                    for _ in range(2):
                        viewer.begin_frame(rendered/plan["fps"])
                        viewer.log_state(state)
                        viewer.log_lines("cad_shell", starts, ends, (.45, .65, .75))
                        viewer.end_frame()
                    panels.append(viewer.get_frame().numpy()[:, :, :3])
                combined = np.concatenate(panels, axis=1)
                encoder.stdin.write(combined.tobytes())
                if index == phase["frames"]-1 and phase["phase"] == "commit":
                    Image.fromarray(combined).save(output/f"arrival_{phase['arrival']:02d}.png")
                if rendered == plan["frames"]-1:
                    Image.fromarray(combined).save(output/"final_perspective.png")
                rendered += 1
                if rendered % 200 == 0:
                    print(json.dumps(dict(rendered=rendered, total=plan["frames"])), flush=True)
        encoder.stdin.close()
        if encoder.wait():
            raise RuntimeError("ffmpeg encoding failed")
    finally:
        if encoder.poll() is None:
            if encoder.stdin and not encoder.stdin.closed:
                encoder.stdin.close()
            encoder.terminate(); encoder.wait()
        viewer.close()
    return dict(status="rendered", video=str(video), frames=rendered,
                camera_world_m=camera.tolist(), target_world_m=target.tolist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=HERE/"result_seed73_v2.json")
    parser.add_argument("--output", type=Path, default=HERE/"render_v2_newton")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=800, help="Per panel")
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument("--render", action="store_true", help="Explicitly create CUDA/GL viewer and render")
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    if not args.output.resolve().is_relative_to(HERE) or args.output.resolve() == HERE:
        parser.error("Output must be a subdirectory of push_compare")
    if args.fps <= 0 or args.width <= 0 or args.height <= 0 or args.width % 2 or args.height % 2:
        parser.error("Positive fps and positive even image dimensions required")
    report = json.loads(args.input.read_text()); plan = prepare(report, args.fps)
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = dict(status="cpu_prepared_not_rendered", input=str(args.input.resolve()),
                    layout=dict(left="no_push", right="ideal_exact_slide", view="Same 3D oblique perspective in both panels; not orthographic/three-view", width=2*args.width, height=args.height),
                    source="Native Newton make_model + ViewerGL; existing CAD container edges and exact proxy cuboid dimensions; no robot",
                    color="Same per-box color in both panels; physical box size follows saved current arrival, not an inflated model",
                    sequence="occupied_before -> smooth single-box movement with all other boxes frozen -> current box at route start -> current_route -> commit",
                    future_visibility="All bodies initially hidden; only committed boxes and the current inserting box are visible; recorded arrivals are used only to allocate correctly sized hidden assets",
                    timing="Presentation interpolation only; phase durations synchronized between panels; not planner/arm/push execution time",
                    dynamics=False, robot=False, show_robot=False, contact_forces=False,
                    ideal_commanded_equals_actual_translation=True, gpu_started=args.render,
                    playback_is_saved_geometry_not_live_planning=True, plan=plan)
    if args.render:
        metadata.update(render(report, plan, args.output, args.device, args.width, args.height))
    (args.output/"render.json").write_text(json.dumps(metadata, indent=2)+"\n")
    print(json.dumps({k: v for k, v in metadata.items() if k != "plan"}, indent=2))
    print(json.dumps(dict(frames=plan["frames"], duration_s=plan["duration_s"])))


if __name__ == "__main__":
    main()
