"""Pure geometric proposals: caller owns acceptance and occupancy commit."""
from dataclasses import asdict
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "experiments/s20_greedy_async_20261023"))
import numpy as np
from compact_prior import CompactPrior
from compare import Box, p
try:
    from .support_geometry_v2 import supported as strict_supported
    from .run_compare import support_fraction, metrics as base_metrics, choose, proposals
except ImportError:
    from support_geometry_v2 import supported as strict_supported
    from run_compare import support_fraction, metrics as base_metrics, choose, proposals


class StrictCompactPrior(CompactPrior):
    def __init__(self, cfg, hull):
        super().__init__(cfg, hull, use_endpoint_sphere=True, max_box_top_m=1.)
        self.horizontal_first = True
        self.contact_first = True

    def supported(self, b, placed):
        return strict_supported(b, placed, self.hull)

    def candidates(self, dims, placed, profile="depth"):
        rows, timing = super().candidates(dims, placed, profile)
        timing["support"] = "Floor or >=80% exact coplanar union area plus center projection in actual contact union; numerical height tolerance 1e-7m"
        return rows, timing


def exact_contacts(boxes, hull):
    pair_area = wall_area = 0.
    for i, b in enumerate(boxes):
        for axis in range(3):
            others = [k for k in range(3) if k != axis]
            for v in boxes[i+1:]:
                if min(abs(b.pos[axis]+b.dims[axis]-v.pos[axis]),
                       abs(v.pos[axis]+v.dims[axis]-b.pos[axis])) <= p.EPS:
                    pair_area += float(np.prod([max(0., min(b.pos[k]+b.dims[k], v.pos[k]+v.dims[k])
                                                               -max(b.pos[k], v.pos[k])) for k in others]))
            for face, wall in ((b.pos[axis], 0.), (b.pos[axis]+b.dims[axis], (hull.length, hull.width, hull.height)[axis])):
                if abs(face-wall) <= p.EPS:
                    wall_area += float(np.prod([b.dims[k] for k in others]))
    return pair_area, wall_area


def metrics(boxes, prior):
    result = base_metrics(boxes, prior)
    pair, wall = exact_contacts(boxes, prior.hull)
    result.update(exact_neighbor_contact_area_m2=pair, exact_wall_contact_area_m2=wall,
                  exact_contact_area_m2=pair+wall)
    if boxes:
        result["exact_support_fraction"] = {key: min(1., value) for key, value in result["exact_support_fraction"].items()}
        result["minimum_exact_support_fraction"] = min(result["exact_support_fraction"].values())
    return result


def move_rejection(old, new, others, prior):
    if not prior.valid(new, others):
        return "moved_endpoint_invalid_including_strict_support"
    if not p.path_clear(old.dims, [old.pos, new.pos], others, prior.hull):
        return "push_swept_collision_or_hull"
    axis = next(k for k in (0, 1) if abs(new.pos[k]-old.pos[k]) > p.EPS)
    delta = new.pos[axis]-old.pos[axis]
    # For one axis-aligned slide, contact-union area is piecewise linear.
    # Edge coincidences delimit pieces; centers also delimit center-support changes.
    times = {0., 1.}
    for v in others:
        for fixed in (v.pos[axis], v.pos[axis]+v.dims[axis]/2, v.pos[axis]+v.dims[axis]):
            for moving in (old.pos[axis], old.pos[axis]+old.dims[axis]/2, old.pos[axis]+old.dims[axis]):
                t = (fixed-moving)/delta
                if 0 < t < 1:
                    times.add(t)
    boundaries = sorted(times)
    times.update((a+b)/2 for a, b in zip(boundaries, boundaries[1:]))
    for t in sorted(times):
        pos = tuple(a+t*(b-a) for a, b in zip(old.pos, new.pos))
        moving = Box(old.id, old.dims, pos)
        if not prior.supported(moving, others):
            return "moving_box_loses_80pct_or_center_support_during_slide"
        for frozen in others:
            supports = [v for v in others if v.id != frozen.id]+[moving]
            if not prior.supported(frozen, supports):
                return "frozen_box_loses_80pct_or_center_support_during_slide"
    return None


def plan_current(currentdims, placed, hull, cfg, *, allow_push=False, label="current", max_shift=.05):
    """Return candidate Box, cargo route, movement dict and copied proposed_placed.

    Input is an iterable of upstream Box objects. Nothing is committed or mutated.
    Only the caller should replace occupancy after accepting movement AND candidate.
    All proposal comparisons use only the current item and committed occupancy.
    """
    if not 0 < max_shift <= .05:
        raise ValueError("max_shift must be in (0, .05]m")
    boxes = [Box(b.id, tuple(b.dims), tuple(b.pos)) for b in placed]
    prior = StrictCompactPrior(cfg, hull)
    if not all(prior.valid(b, [v for v in boxes if v.id != b.id]) for b in boxes):
        raise ValueError("Input occupancy fails v2 geometry/support constraints; no implicit repair")
    start = time.perf_counter()
    direct, route, timing = choose(label, currentdims, boxes, prior)
    selected = direct
    baseline = metrics(boxes+([direct] if direct else []), prior)
    best_key = (baseline["envelope_volume_m3"], -baseline["exact_contact_area_m2"])
    best_boxes = boxes; movement = None; winning = None; trials = []
    if allow_push:
        for old, new in proposals(boxes, prior, max_shift):
            began = time.perf_counter(); others = [b for b in boxes if b.id != old.id]
            reason = move_rejection(old, new, others, prior)
            row = dict(before=asdict(old), after=asdict(new), committed=False,
                       commanded_displacement_m=(np.array(new.pos)-old.pos).tolist())
            if reason is None:
                changed = [new if b.id == old.id else b for b in boxes]
                candidate, candidate_route, search = choose(label, currentdims, changed, prior)
                after = metrics(changed+([candidate] if candidate else []), prior)
                row.update(current_choice=asdict(candidate) if candidate else None, search=search,
                           metrics_after_push=metrics(changed, prior), metrics_with_current=after)
                envelope = after["envelope_volume_m3"]; contact = after["exact_contact_area_m2"]
                key = (round(envelope, 9), -round(contact, 9))
                if candidate is None:
                    reason = "no_current_candidate_after_move"
                elif direct is not None and envelope > baseline["envelope_volume_m3"]+1e-9:
                    reason = "would_worsen_envelope_with_same_current_acceptance"
                elif direct is not None and not (envelope < baseline["envelope_volume_m3"]-1e-9
                                                  or contact > baseline["exact_contact_area_m2"]+1e-9):
                    reason = "no_current_compactness_or_exact_contact_gain"
                elif winning is not None and key >= best_key:
                    reason = "inferior_to_best_proposal"
                else:
                    if winning is not None:
                        trials[winning]["rejection_reason"] = "superseded_by_better_proposal"
                    best_key = key; winning = len(trials); best_boxes = changed
                    selected = candidate; route = candidate_route
                    movement = dict(box_id=old.id, before=asdict(old), after=asdict(new),
                                    commanded_displacement_m=row["commanded_displacement_m"],
                                    ideal_actual_displacement_m=row["commanded_displacement_m"],
                                    reason="current_acceptance_gain" if direct is None else "current_envelope_or_exact_contact_gain")
            row.update(rejection_reason=reason, elapsed_s=time.perf_counter()-began)
            trials.append(row)
    return dict(movement=movement, candidate=selected, route=route, proposed_placed=list(best_boxes),
                baseline_candidate=asdict(direct) if direct else None, baseline_metrics_with_current=baseline,
                direct_search=timing, selected_trial=winning, trials=trials,
                elapsed_s=time.perf_counter()-start, future_arrivals_used=0,
                acceptance_required=True, input_occupancy_mutated=False)


def propose_local_rearrangement(currentdims, placed, hull, cfg, **kwargs):
    """compact_prior adapter; proposal only. candidate/route may be None."""
    return plan_current(currentdims, placed, hull, cfg, allow_push=True, **kwargs)
