"""Isolated current-arrival packing comparison. No robot commands or RL.

Reuses the classmate's hull, support, candidate coordinates and swept-AABB
insertion predicate. All methods receive exactly the same geometric pool.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import itertools
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from scipy.sparse.csgraph import connected_components
from scipy.sparse import coo_matrix
import yaml

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = Path(os.environ.get("S20_PACKING_UPSTREAM",ROOT / "reproductions/ludan_greedy_20261023"))
sys.path.insert(0, str(UPSTREAM / "policy"))
import online_packing_real as p
from online_packing_demo import Box, coordinates, collides, fully_supported

SCENE = UPSTREAM / "deployment_ws/config/scene_tf.loading_site.yaml"


def top_surface_model(points_world, ground_plane, plane_tolerance_m=.008):
    """Input is a selected top-surface cloud, not an image-wide AABB.

    RANSAC removes unrelated points; metric projection removes perspective.
    The footprint is not inflated. Ground normal fixes upward orientation.
    """
    import cv2
    pts = np.asarray(points_world, dtype=float)
    pts = pts[np.isfinite(pts).all(axis=1)]
    ground = np.asarray(ground_plane, dtype=float)
    ground /= np.linalg.norm(ground[:3])
    rng = np.random.default_rng(23)
    best = np.zeros(len(pts), dtype=bool)
    for _ in range(80):
        tri = pts[rng.choice(len(pts), 3, replace=False)]
        normal = np.cross(tri[1]-tri[0], tri[2]-tri[0])
        if np.linalg.norm(normal) < 1e-9:
            continue
        normal /= np.linalg.norm(normal)
        if abs(normal @ ground[:3]) < .9:
            continue
        mask = np.abs((pts-tri[0]) @ normal) <= plane_tolerance_m
        if mask.sum() > best.sum():
            best = mask
    surface = pts[best]
    # A coplanar luggage label/remote patch can survive a plane fit. Keep the
    # connected selected surface, not every inlier across the whole scene.
    edges = cKDTree(surface).query_pairs(.03, output_type="ndarray")
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(surface), len(surface)))
    _, components = connected_components(graph, directed=False)
    surface = surface[components == np.argmax(np.bincount(components))]
    center = surface.mean(axis=0)
    _, _, vt = np.linalg.svd(surface-center, full_matrices=False)
    normal = vt[-1]
    if normal @ ground[:3] < 0:
        normal = -normal
    u = vt[0]
    v = np.cross(normal, u)
    plane_xy = np.c_[(surface-center)@u, (surface-center)@v]
    rect = cv2.minAreaRect(plane_xy.astype(np.float32))
    corners2 = cv2.boxPoints(rect).astype(float)
    corners = center + corners2[:, 0, None]*u + corners2[:, 1, None]*v
    top_center = corners.mean(axis=0)
    height = float(np.median(surface @ ground[:3] + ground[3]))
    bottom = corners - ground[:3]*height
    all_points = np.r_[corners, bottom]
    sides = sorted(rect[1], reverse=True)
    edges3 = np.roll(corners,-1,axis=0)-corners
    axis = edges3[np.argmax(np.linalg.norm(edges3,axis=1))]
    axis /= np.linalg.norm(axis)
    if axis[0]<-1e-8 or abs(axis[0])<=1e-8 and axis[1]<0:axis=-axis
    box_tf = np.eye(4)
    box_tf[:3,:3] = np.column_stack((axis,np.cross(normal,axis),normal))
    box_tf[:3,3] = top_center-ground[:3]*height/2
    return dict(top_center_world_m=top_center.tolist(), normal_world=normal.tolist(),
                long_axis_world=axis.tolist(),short_axis_world=np.cross(normal,axis).tolist(),
                yaw_world_rad=float(np.arctan2(axis[1],axis[0])),yaw_world_deg=float(np.degrees(np.arctan2(axis[1],axis[0]))),
                rectangle_yaw_symmetry_deg=180,
                top_corners_world_m=corners.tolist(), dimensions_m=[*sides, height],
                T_world_box=box_tf.tolist(),
                aabb_min_world_m=all_points.min(axis=0).tolist(),
                aabb_max_world_m=all_points.max(axis=0).tolist(),
                plane_inliers=int(best.sum()), connected_surface_points=len(surface), points=len(pts),
                plane_rms_m=float(np.sqrt(np.mean(((surface-center)@normal)**2))),
                inflation_m=0., source="selected metric RGB-D points")


def pool(dims, placed, hull):
    # Contact coordinates replace a dense XYZ voxel sweep; Z is floor/support.
    result = []
    for size in sorted({tuple(dims), (dims[1], dims[0], dims[2])}):
        lx, ly, lz = size
        xs = coordinates(hull.length, lx, placed, 0)
        ys = sorted(set(coordinates(hull.width, ly, placed, 1)) |
                    {round(hull.flat_y-ly, 3), round(hull.door_y0, 3),
                     round(hull.door_y1-ly, 3)})
        zs = sorted({0.} | {round(b.pos[2]+b.dims[2], 3) for b in placed})
        for x, y, z in itertools.product(xs, ys, zs):
            box = Box("current", size, (x, y, z))
            if (hull.fits(box) and not any(collides(box, b) for b in placed)
                    and (hull.floor_supports(box) or fully_supported(box, placed))):
                result.append(box)
    return result


def contact_area(box, placed, hull):
    total = 0.
    for axis in (0, 1):
        others = [k for k in range(3) if k != axis]
        area = float(np.prod([box.dims[k] for k in others]))
        if abs(box.pos[axis]) < .002 or abs(box.pos[axis]+box.dims[axis]-
                                            (hull.length, hull.width)[axis]) < .002:
            total += area
        for b in placed:
            if min(abs(box.pos[axis]+box.dims[axis]-b.pos[axis]),
                   abs(b.pos[axis]+b.dims[axis]-box.pos[axis])) < .002:
                total += float(np.prod([max(0., min(box.pos[k]+box.dims[k],
                    b.pos[k]+b.dims[k])-max(box.pos[k], b.pos[k])) for k in others]))
    return total


def old_score(box, placed, hull, profile):
    depth = (box.pos[0]+box.dims[0]/2)/hull.length
    edge_gap = min(box.pos[1], hull.width-box.pos[1]-box.dims[1],
                   *(abs(box.pos[1]-b.pos[1]-b.dims[1]) for b in placed))
    height = box.pos[2]/hull.height
    # Same three terms as the old spatial packing_score, adapted to this hull.
    return 1.5*(1-depth)+2*(-np.expm1(-edge_gap/.15))+1.2*height


def choose(dims, placed, hull, method, profile="flat"):
    began = time.perf_counter()
    rows = pool(dims, placed, hull)
    generated = time.perf_counter()
    if method == "old_score":
        rows.sort(key=lambda b: old_score(b, placed, hull, profile))
    else:
        def key(b):
            # Depth is lexicographic, not traded away for cheap short motion.
            depth = round(b.pos[0]+b.dims[0], 3)
            layers = b.pos[2]/dims[2]
            if profile == "stack":
                height_cost = -layers
            elif profile == "pair":
                height_cost = (layers-1)**2
            else:
                height_cost = layers
            if method == "greedy_layer":
                return (height_cost, -depth, -contact_area(b, placed, hull), b.pos[1])
            return (-depth, height_cost, -contact_area(b, placed, hull), b.pos[1])
        rows.sort(key=key)
    ranked = time.perf_counter()
    route_checks, feasible = 0, []
    for box in rows:
        route_checks += 1
        route = p.route_to(box, placed, hull)
        if route is not None:
            feasible.append((box, route))
            if method != "greedy_top4" or len(feasible) == 4:
                break
    if feasible and method == "greedy_top4":
        # Only compare routes at the same preferred depth and height family.
        first = feasible[0][0]
        tied = [(b, r) for b, r in feasible if
                abs(b.pos[0]+b.dims[0]-first.pos[0]-first.dims[0]) < .002
                and abs(b.pos[2]-first.pos[2]) < .002]
        feasible = sorted(tied, key=lambda br: p.route_length(br[1]))
    elapsed = time.perf_counter()
    return dict(box=None if not feasible else asdict(feasible[0][0]),
                route=None if not feasible else feasible[0][1],
                candidates=len(rows), route_checks=route_checks,
                generation_s=generated-began, rank_s=ranked-generated,
                route_s=elapsed-ranked, total_s=elapsed-began)


def worker(request):
    hull = p.Hull.from_dict(request["hull"])
    placed = [Box(**{k: tuple(v) if k != "id" else v for k, v in b.items()})
              for b in request["placed"]]
    start = time.perf_counter()
    value = choose(request["dims"], placed, hull, request["method"])
    return dict(value=value, start=start, finish=time.perf_counter(),
                scene_revision=request["scene_revision"],
                attachment=request["T_tool_box"], staging=request["staging_q"])


class AsyncPlacement:
    """One warm worker; no SDK, only current-object placement references."""
    def __init__(self):
        self.executor = ProcessPoolExecutor(max_workers=1)
        self.future = None

    def start(self, request):
        self.request = json.loads(json.dumps(request))
        self.sent = time.perf_counter()
        self.future = self.executor.submit(worker, self.request)

    def consume(self, actual):
        result = self.future.result()
        geometry_changed = any(actual[k] != self.request[k] for k in
                               ("scene_revision", "dims", "placed", "hull", "method"))
        if geometry_changed:
            return dict(status="replan_geometry", executable=False)
        motion_changed = any(actual[k] != self.request[k] for k in
                             ("T_tool_box", "staging_q"))
        return dict(status="refine_motion" if motion_changed else "reference_ready",
                    executable=False, result=result,
                    wait_after_staging_s=max(0., result["finish"]-actual["arrived_at"]))

    def close(self):
        self.executor.shutdown(wait=True)


def calibration():
    cfg = yaml.safe_load(SCENE.read_text())
    old = yaml.safe_load((ROOT/"records/S20/exports/scene_tf.yaml").read_text())
    get = lambda c: next(t for t in c["static_transforms"] if t["child"] == "container_link")
    a, b = get(old), get(cfg)
    r1, r2 = [Rotation.from_euler("xyz", t["rotation_rpy"]) for t in (a, b)]
    return dict(frame="world -> container_link", old=a, new=b,
                translation_delta_world_m=(np.array(b["translation"])-a["translation"]).tolist(),
                rotation_delta_deg=float(np.degrees((r1.inv()*r2).magnitude())),
                old_user_minus6cm_to_new_delta_m=[0., -.11, .16],
                source=str(SCENE.relative_to(ROOT)),
                status="upstream site configuration; not independently re-surveyed here")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent/"results")
    parser.add_argument("--seeds", type=int, default=12)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    hull = p.Hull.from_config(yaml.safe_load(SCENE.read_text()))
    results = []
    for seed in range(args.seeds):
        rng = np.random.default_rng(seed+1023)
        sizes = (np.tile([.55,.40,.25], (8,1))*rng.uniform(.95,1.05,(8,3))).tolist()
        for method in ("old_score", "greedy_first", "greedy_top4", "greedy_layer"):
            placed, steps = [], []
            for i, dims in enumerate(sizes):
                step = choose(dims, placed, hull, method)
                step.update(incoming=i+1, dims=dims)
                if step["box"]:
                    step["box"]["id"] = f"B{i+1}"
                    placed.append(Box(**step["box"]))
                steps.append(step)
            results.append(dict(seed=seed+1023, method=method, count=len(placed), steps=steps))
    cases = []
    for profile in ("flat", "pair", "stack"):
        placed, steps = [], []
        for i in range(3):
            step = choose([.55,.40,.25], placed, hull, "greedy_first", profile)
            if step["box"]:
                step["box"]["id"] = f"B{i+1}"
                placed.append(Box(**step["box"]))
            steps.append(step)
        cases.append(dict(profile=profile, steps=steps))
    rng = np.random.default_rng(23)
    xy = rng.uniform([-.275,-.2],[.275,.2],(2500,2))
    pts = np.c_[xy, .25+rng.normal(0,.0015,len(xy))]
    pts = np.r_[pts, rng.uniform([-.6,-.5,0],[.6,.5,.5],(150,3))]
    model = top_surface_model(pts, [0.,0.,1.,0.])
    # Foreground is actual local RGB-D model work, NOT a fabricated robot Pick.
    request = dict(hull=hull.as_dict(), placed=[], dims=[.55,.4,.25],
                   method="greedy_first", scene_revision=0,
                   T_tool_box=np.eye(4).tolist(), staging_q=[0.]*6)
    async_rows = []
    async_plan = AsyncPlacement()
    for j in range(4):
        t0 = time.perf_counter()
        async_plan.start(request)
        foreground_start = time.perf_counter()
        top_surface_model(pts, [0.,0.,1.,0.])
        arrived = time.perf_counter()
        value = async_plan.consume(dict(request, arrived_at=arrived))
        value.update(trial=j, foreground_s=arrived-foreground_start,
                     wall_s=time.perf_counter()-t0, includes_worker_startup=j==0)
        async_rows.append(value)
    changed = dict(request, scene_revision=1, arrived_at=time.perf_counter())
    invalidation = async_plan.consume(changed)
    async_plan.close()
    summary = []
    for method in ("old_score", "greedy_first", "greedy_top4", "greedy_layer"):
        rows = [r for r in results if r["method"] == method]
        steps = [s for r in rows for s in r["steps"]]
        summary.append(dict(method=method, arrivals=len(steps), placed=sum(r["count"] for r in rows),
                            median_ms=float(np.median([s["total_s"] for s in steps])*1000),
                            p95_ms=float(np.quantile([s["total_s"] for s in steps],.95)*1000),
                            route_checks=sum(s["route_checks"] for s in steps)))
    report = dict(scope="GEOMETRY + straight-segment cargo swept AABB ONLY; no arm/dynamics/RL",
                  upstream_commit="1f0600802ed6c4b7beb4b4187aba4c2f299449b6",
                  flow="current arrival only; unknown future; no item reordering or relocation",
                  timings="actual local wall clock; process startup separated; no hardware cycle claim",
                  calibration=calibration(), summary=summary, cases=cases, runs=results,
                  aabb_synthetic=model, async_trials=async_rows, invalidation=invalidation,
                  limitations=["top cloud and ground plane assumed supplied by segmentation/calibration",
                               "support complete; no footprint overhang relaxation in this comparison",
                               "collision tolerance unchanged; geometric route not arm feasibility"])
    (args.output/"comparison.json").write_text(json.dumps(report, indent=2))
    np.savez_compressed(args.output/"aabb_synthetic.npz", points=pts)
    print(json.dumps(dict(summary=summary, aabb_dims=model["dimensions_m"],
                         async_wait_s=[r["wait_after_staging_s"] for r in async_rows]), indent=2))


if __name__ == "__main__":
    main()
