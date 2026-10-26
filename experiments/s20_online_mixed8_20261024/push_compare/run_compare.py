"""CPU current-arrival geometry comparison; ideal translations, no robot/physics."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
for name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[name] = "1"
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "experiments/s20_greedy_async_20261023"))
import numpy as np
import yaml
from compact_prior import CompactPrior
from compare import Box, SCENE, p


def support_fraction(b, others, hull):
    if hull.floor_supports(b):
        return 1.0
    supports = [v for v in others if abs(v.pos[2]+v.dims[2]-b.pos[2]) < p.EPS]
    axes = [sorted({b.pos[k], b.pos[k]+b.dims[k]} | {
        max(b.pos[k], min(b.pos[k]+b.dims[k], v.pos[k]+side*v.dims[k]))
        for v in supports for side in (0, 1)}) for k in (0, 1)]
    area = 0.0
    for x0, x1 in zip(axes[0], axes[0][1:]):
        for y0, y1 in zip(axes[1], axes[1][1:]):
            if any(v.pos[0]-p.EPS <= (x0+x1)/2 <= v.pos[0]+v.dims[0]+p.EPS
                   and v.pos[1]-p.EPS <= (y0+y1)/2 <= v.pos[1]+v.dims[1]+p.EPS for v in supports):
                area += (x1-x0)*(y1-y0)
    return area/(b.dims[0]*b.dims[1])


def metrics(boxes, prior):
    if not boxes:
        return dict(count=0, packed_volume_m3=0., envelope_volume_m3=0., envelope_utilization=0.)
    lo = np.array([b.pos for b in boxes]); hi = lo+np.array([b.dims for b in boxes])
    envelope = float(np.prod(hi.max(0)-lo.min(0)))
    volume = sum(float(np.prod(b.dims)) for b in boxes)
    # Integrate the actual chamfer cross-section below the 1m task ceiling.
    h = prior.hull; ceiling = min(1., h.height)
    clipped_rise = min(ceiling, h.slope_rise)
    cross_loss = (h.width-h.flat_y)/h.slope_rise*(ceiling*clipped_rise-clipped_rise**2/2)
    task_volume = h.length*(h.width*ceiling-cross_loss)
    contacts = [prior.contacts(b, [v for v in boxes if v.id != b.id]) for b in boxes]
    supports = {b.id: support_fraction(b, [v for v in boxes if v.id != b.id], h) for b in boxes}
    return dict(count=len(boxes), packed_volume_m3=volume, task_container_volume_m3=task_volume,
                task_volume_utilization=volume/task_volume, full_hull_volume_utilization=volume/h.volume,
                envelope_min_m=lo.min(0).tolist(), envelope_max_m=hi.max(0).tolist(),
                envelope_volume_m3=envelope, envelope_utilization=volume/envelope,
                neighbor_soft_contact_area_m2=sum(c[0] for c in contacts)/2,
                wall_soft_contact_area_m2=sum(c[1] for c in contacts),
                exact_support_fraction=supports, minimum_exact_support_fraction=min(supports.values()),
                accepted_support_rule_all=all(prior.supported(b, [v for v in boxes if v.id != b.id]) for b in boxes))


def choose(label, dims, boxes, prior):
    start = time.perf_counter(); rows, timing = prior.candidates(dims, boxes)
    rejects = Counter()
    for b in rows:
        route = p.route_to(b, boxes, prior.hull)
        if route is None:
            rejects["no_swept_cargo_insertion_route"] += 1
            continue
        b = Box(label, b.dims, tuple(route[-1]))
        if not prior.valid(b, boxes) or not p.path_clear(b.dims, route, boxes, prior.hull):
            rejects["rounded_route_endpoint_invalid"] += 1
            continue
        return b, route, dict(candidates=timing, rejection_counts=dict(rejects), elapsed_s=time.perf_counter()-start)
    return None, None, dict(candidates=timing, rejection_counts=dict(rejects), elapsed_s=time.perf_counter()-start,
                           reason="no_valid_candidate" if not rows else "all_routes_rejected")


def objective(boxes, current, prior):
    m = metrics(boxes+([current] if current else []), prior)
    return (int(current is None), round(current.pos[2], 6) if current else 0.,
            round(m["envelope_volume_m3"], 6), -round(m.get("neighbor_soft_contact_area_m2", 0.), 6))


def proposals(boxes, prior, max_shift):
    for b in sorted(boxes, key=lambda b: (b.pos[2], b.id)):
        for axis in (0, 1):
            targets = {b.pos[axis]+sign*distance for sign in (-1, 1) for distance in (.01, .03, max_shift)}
            targets |= {0., (prior.hull.length, prior.hull.width)[axis]-b.dims[axis]}
            for v in boxes:
                if v.id != b.id:
                    targets |= {v.pos[axis]-b.dims[axis], v.pos[axis]+v.dims[axis]}
            for target in sorted(targets):
                delta = target-b.pos[axis]
                if 1e-6 < abs(delta) <= max_shift+1e-8:
                    pos = list(b.pos); pos[axis] = target
                    yield b, Box(b.id, b.dims, tuple(pos))


def move_rejection(old, new, others, prior):
    # Conservative dependency veto includes the inherited 5cm support allowance.
    for upper in others:
        if (-p.EPS <= upper.pos[2]-old.pos[2]-old.dims[2] <= .05+p.EPS
                and all(min(old.pos[k]+old.dims[k]+.05, upper.pos[k]+upper.dims[k])
                        > max(old.pos[k]-.05, upper.pos[k])+p.EPS for k in (0, 1))):
            return "upper_support_dependency_no_linked_motion"
    if not prior.valid(new, others):
        return "moved_endpoint_hull_collision_support_height_or_sphere"
    if not p.path_clear(old.dims, [old.pos, new.pos], others, prior.hull):
        return "push_swept_collision_or_hull"
    lower = tuple(min(a, b) for a, b in zip(old.pos, new.pos))
    dims = tuple(d+abs(a-b) for d, a, b in zip(old.dims, old.pos, new.pos))
    swept = Box("swept_support", dims, lower)
    if not (prior.hull.floor_supports(swept) or p.fully_supported(swept, others)):
        return "push_path_not_exactly_supported"
    if not all(prior.valid(v, [w for w in others if w.id != v.id]+[new]) for v in others):
        return "stationary_box_support_or_validity_lost"
    return None


def run(seed, count, enabled, prior, max_shift):
    rng = np.random.default_rng(seed); boxes = []; steps = []
    for index in range(count):
        # Draw only the current arrival, identically in both independent runs.
        label = f"B{index+1}"; size = int(rng.integers(0, 2))
        dims = (np.array(((.55, .40, .25), (.65, .45, .30))[size])*rng.uniform(.9, 1.1, 3)).round(3).tolist()
        start = time.perf_counter(); before = list(boxes)
        selected, route, direct = choose(label, dims, boxes, prior)
        baseline_key = objective(boxes, selected, prior); best_key = baseline_key
        trials = []; winning = None
        if enabled:
            for old, new in proposals(boxes, prior, max_shift):
                began = time.perf_counter(); others = [b for b in boxes if b.id != old.id]
                reason = move_rejection(old, new, others, prior)
                row = dict(box_id=old.id, before=asdict(old), after=asdict(new),
                           commanded_displacement_m=(np.array(new.pos)-old.pos).tolist())
                if reason is None:
                    changed = [new if b.id == old.id else b for b in boxes]
                    candidate, candidate_route, search = choose(label, dims, changed, prior)
                    key = objective(changed, candidate, prior)
                    row.update(current_choice=asdict(candidate) if candidate else None,
                               candidate_search=search, metrics_after_push=metrics(changed, prior), objective=key)
                    if key < best_key:
                        best_key = key; winning = len(trials)
                        selected, route = candidate, candidate_route
                    else:
                        reason = "no_strict_improvement_over_best_current_prefix"
                row.update(rejection_reason=reason, elapsed_s=time.perf_counter()-began, committed=False)
                trials.append(row)
            if winning is not None:
                trials[winning]["committed"] = True
                moved = Box(**trials[winning]["after"])
                boxes = [moved if b.id == moved.id else b for b in boxes]
        after_push = list(boxes)
        if selected:
            boxes.append(selected)
        steps.append(dict(id=label, nominal_size_inch=(20, 24)[size], source_dimensions_m=dims,
                          future_arrivals_used=0, occupied_before=[asdict(b) for b in before],
                          occupied_after_push=[asdict(b) for b in after_push],
                          metrics_before_push=metrics(before, prior), metrics_after_push=metrics(after_push, prior),
                          direct_search=direct, push_trials=trials, committed_push_trial=winning,
                          current_choice=asdict(selected) if selected else None, current_route=route,
                          yaw_deg=None if selected is None else (0 if tuple(dims) == selected.dims else 90),
                          rejection_reason=None if selected else "no_feasible_current_placement_in_searched_pool",
                          metrics_after_arrival=metrics(boxes, prior), elapsed_s=time.perf_counter()-start))
        print(json.dumps(dict(mode="push" if enabled else "no_push", arrival=index+1,
                              accepted=len(boxes), push=winning is not None,
                              elapsed_s=steps[-1]["elapsed_s"])), flush=True)
    return dict(mode="push" if enabled else "no_push", steps=steps, final_boxes=[asdict(b) for b in boxes],
                metrics=metrics(boxes, prior), total_planning_s=sum(s["elapsed_s"] for s in steps),
                committed_pushes=sum(s["committed_push_trial"] is not None for s in steps))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=73)
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--max-shift", type=float, default=.05)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("result_seed73_v1.json"))
    args = parser.parse_args()
    if args.output.resolve().parent != Path(__file__).resolve().parent:
        parser.error("Output must remain in push_compare")
    if args.output.exists():
        parser.error("Refusing to overwrite existing results; choose a new filename")
    if not 0 < args.max_shift <= .05:
        parser.error("max-shift must be in (0, .05]m")
    cfg = yaml.safe_load(SCENE.read_text()); hull = p.Hull.from_config(cfg)
    prior = CompactPrior(cfg, hull, use_endpoint_sphere=True, max_box_top_m=1.)
    prior.horizontal_first = True; prior.contact_first = True
    runs = [run(args.seed, args.count, enabled, prior, args.max_shift) for enabled in (False, True)]
    a, b = [r["metrics"] for r in runs]
    summary = dict(accepted_no_push=a["count"], accepted_push=b["count"],
                   count_gain=b["count"]-a["count"], packed_volume_gain_m3=b["packed_volume_m3"]-a["packed_volume_m3"],
                   envelope_utilization_gain=b["envelope_utilization"]-a["envelope_utilization"],
                   committed_pushes=runs[1]["committed_pushes"],
                   planning_s={r["mode"]: r["total_planning_s"] for r in runs})
    report = dict(seed=args.seed, count=args.count, summary=summary, runs=runs,
                  scene_config=str(SCENE), scene_config_snapshot=cfg, hull=hull.as_dict(),
                  endpoint_sphere=dict(center_world_m=prior.center.tolist(), radius_m=1.7),
                  constraints=dict(max_top_m=1., yaw_deg=[0, 90], max_axis_push_m=args.max_shift),
                  arrival_distribution="IID 20/24 equiprobable; independent axes uniform +/-10%; 1mm rounding; only current draw available",
                  support_rule="Inherited CompactPrior 5cm edge/height allowance, supported center; exact area separately reported",
                  contact_metric="Inherited distance-weighted face area: 2cm sides, 5cm underside; not physical contact force",
                  selection="Same CompactPrior contact-first placement for both; push compares current success, lower current z, smaller envelope, greater soft contact",
                  search_scope="One axis translation of one existing box per arrival, low layers enumerated first; finite 1/3/5cm and flush-face targets; no future lookahead; no linked motion",
                  limitations=["Ideal commanded displacement equals actual displacement; no force/friction/contact dynamics",
                               "Swept cargo AABB only; no arm IK, tool access or pushing end-effector path",
                               "1.7m endpoint sphere is not a robot reachability certificate",
                               "Upper-dependent boxes vetoed conservatively; coupled layer motion future work only",
                               "No claim of global optimum, physical stability or generalization from eight arrivals"],
                  robot_execution=False, gpu_used=False, physics_execution=False)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
