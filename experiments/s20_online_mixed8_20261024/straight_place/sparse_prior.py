"""Current-item sparse edge/contact prior; no voxel grid or future arrivals.

Same (boxes, stats) interface as CompactPrior. Geometry/support/sphere remain
inherited; endpoint distance is a ranking heuristic, not native IK feasibility.
"""
import time

import numpy as np

from compact_prior import CompactPrior
from compare import Box, p


class SparseCompactPrior(CompactPrior):
    def __init__(self, cfg, hull, *, max_candidates=10, ordering='greedy', neighbor_gap_m=0., **kwargs):
        super().__init__(cfg, hull, **kwargs)
        if ordering not in ('greedy', 'reach', 'diverse'):
            raise ValueError('ordering must be greedy, reach, or diverse')
        if not 1 <= max_candidates <= 10:
            raise ValueError('max_candidates must be between 1 and 10')
        self.max_candidates = int(max_candidates)
        self.ordering = ordering
        self.row_first = True
        self.horizontal_first = True
        self.neighbor_gap_m=float(neighbor_gap_m)

    def _connected(self, box, placed):
        low = np.asarray(box.pos); high = low+box.dims
        for other in placed:
            olow = np.asarray(other.pos); ohigh = olow+other.dims
            for axis in range(3):
                others = [i for i in range(3) if i != axis]
                if (min(abs(low[axis]-ohigh[axis]), abs(high[axis]-olow[axis])) <= (self.neighbor_gap_m+1e-7 if axis<2 else 1e-7)
                        and np.all(np.minimum(high[others], ohigh[others])
                                   -np.maximum(low[others], olow[others]) > 1e-7)):
                    return True
        return False

    @staticmethod
    def _exposed(box, placed):
        # Conservative same-height door-to-face lane: no lifting over an
        # already committed blocking row to backfill an occluded hole.
        swept = Box('frontier_lane', (box.pos[0]+box.dims[0], *box.dims[1:]),
                    (0., box.pos[1], box.pos[2]))
        return not any(p.collides(swept, other) for other in placed)

    @staticmethod
    def _floor_frontier(box, placed):
        if abs(box.pos[2]) > 1e-7:
            return False
        floor = [b for b in placed if b.pos[2] < box.dims[2]-1e-7]
        if not floor:
            return False
        front = min(b.pos[0] for b in floor)
        return (abs(box.pos[0]+box.dims[0]-front) <= 1e-7
                or any(abs(box.pos[0]-b.pos[0]) <= 1e-7
                       or abs(box.pos[0]+box.dims[0]-b.pos[0]-b.dims[0]) <= 1e-7
                       for b in floor))

    def _families(self, box, placed):
        rear = max((b.pos[0]+b.dims[0] for b in placed), default=self.hull.length)
        front = min((b.pos[0] for b in placed), default=rear)
        end = box.pos[0]+box.dims[0]
        frontier = ('initial' if not placed else 'rear' if end >= rear-.02
                    else 'outward' if end <= front+.02 else 'infill')
        return (('yaw', int(box.dims[0] < box.dims[1])),
                ('height', 'floor' if abs(box.pos[2]) <= 1e-7 else 'stack'),
                ('frontier', frontier))

    def _distance(self, box):
        return min(float(np.linalg.norm(self.top(box)+self.rotation.apply(offset)-self.center))
                   for offset in self.grasp_offsets(box.dims))

    def _rank(self, box, placed, profile, ordering):
        greedy = (round(box.pos[2]/.05), round(box.pos[1]/.02),
                  -round((box.pos[0]+box.dims[0])/.02), self.key(box, placed, profile))
        if ordering != 'reach':
            return greedy
        # Reach is a tie-breaker inside the same low/right/depth band.
        return (*greedy[:3], self._distance(box), greedy[3])

    def _select(self, rows, placed, profile, ordering):
        ranked = sorted(rows, key=lambda b: self._rank(b, placed, profile, ordering))
        selected = ranked[:1]
        # Lowest is a priority, not a geometric veto on a supported higher
        # endpoint when all low endpoints later fail IK or original CAD.
        if selected and self.max_candidates > 1:
            higher = next((b for b in ranked if b.pos[2] > selected[0].pos[2]+1e-7), None)
            if higher is not None:
                selected.append(higher)
        covered = {feature for b in selected for feature in self._families(b, placed)}
        # Best packing candidate stays first. Reserve best representatives of
        # remaining orientation/frontier alternatives, with a higher backup.
        for family in (None, 'yaw', 'height', 'frontier'):
            for box in ranked:
                features = self._families(box, placed)
                needed = not selected if family is None else any(
                    name == family and (name, value) not in covered for name, value in features)
                if needed and box not in selected and len(selected) < self.max_candidates:
                    selected.append(box)
                    covered.update(features)
        if ordering == 'diverse':
            scale = np.array([self.hull.length, self.hull.width, self.hull.height])

            def separation(box):
                center = (np.array(box.pos)+np.array(box.dims)/2)/scale
                return min(float(np.linalg.norm(center-(np.array(other.pos)+np.array(other.dims)/2)/scale))
                           + .2*int(box.dims != other.dims) for other in selected)

            while len(selected) < min(self.max_candidates, len(ranked)):
                # Stable rank order breaks equal-distance ties.
                selected.append(max((b for b in ranked if b not in selected), key=separation))
        else:
            selected.extend(b for b in ranked if b not in selected)
            selected = sorted(selected[:self.max_candidates],
                              key=lambda b: self._rank(b, placed, profile, ordering))
        return sorted(selected, key=lambda b: self._rank(b, placed, profile, ordering))

    def candidates(self, dims, placed, profile='depth', *, ordering=None):
        began = time.perf_counter()
        ordering = self.ordering if ordering is None else ordering
        if ordering not in ('greedy', 'reach', 'diverse'):
            raise ValueError('ordering must be greedy, reach, or diverse')
        dims = tuple(float(v) for v in dims)
        if len(dims) != 3 or not np.isfinite(dims).all() or min(dims) <= 0:
            raise ValueError('dims must contain three positive finite dimensions')
        placed = list(placed)
        h = self.hull
        rows = {}
        tried = set()
        intervals = {}
        rejection = dict(hidden=0, detached=0, higher_level=0, floor_fallback_deferred=0)
        connected = {}
        levels = {0.} | {b.pos[2]+b.dims[2] for b in placed}

        def offer(size, x, y, z, *, clip_x=False):
            if y < self.lateral_clearance_m-1e-7 or y+size[1] > h.width+1e-7:
                return
            if self.max_box_top_m is not None and z+size[2] > self.max_box_top_m+1e-7:
                return
            key = (size, y, z)
            if key not in intervals:
                intervals[key] = self.interval(size, y, z)
            interval = intervals[key]
            if interval is None:
                return
            lo, hi = interval
            x = hi if x is None else min(max(x, lo), hi) if clip_x else x
            if not lo-1e-8 <= x <= hi+1e-8:
                return
            box = Box('current', size, (float(x), float(y), float(z)))
            identity = (box.pos, box.dims)
            if identity in tried:
                return
            tried.add(identity)
            if self.valid(box, placed):
                rows[identity] = box

        for size in sorted({dims, (dims[1], dims[0], dims[2])}):
            right = self.lateral_clearance_m
            if self.lateral_bounds is not None:
                right = max(right, self.lateral_bounds[0])
            if not placed:
                # Only initial right/rear anchors use a short reach-boundary ladder.
                offer(size, None, right, 0.)
                interval = intervals.get((size, right, 0.))
                if interval is not None:
                    for delta in (.03, .08, .16):
                        offer(size, max(interval[0], interval[1]-delta), right, 0.)
                continue
            for other in placed:
                x, y, z = other.pos
                dx, dy, dz = other.dims
                xs = (x, x+dx-size[0], x+(dx-size[0])/2)
                ys = (y, y+dy-size[1], y+(dy-size[1])/2)
                # Constant local combinations per face, never global XYZ products.
                for face_z in sorted({0.} | ({z} if z in levels else set())):
                    for ax in xs[:2]:
                        for ay in (y-size[1]-self.neighbor_gap_m, y+dy+self.neighbor_gap_m):
                            offer(size, ax, ay, face_z, clip_x=True)
                    for ay in ys:
                        offer(size, x-size[0]-self.neighbor_gap_m, ay, face_z)
                for ax in xs:
                    for ay in ys:
                        offer(size, ax, max(right, ay), z+dz, clip_x=True)
                # Aligned floor-frontier fallback, not remote free-space points.
                for ax in xs[:2]:
                    offer(size, ax, y+dy+self.lateral_clearance_m, 0.)
                    offer(size, ax, y-size[1]-self.lateral_clearance_m, 0.)
                offer(size, x-size[0], right, 0.)
        raw_valid = len(rows)
        eligible = []
        for identity, box in rows.items():
            if not self._exposed(box, placed):
                rejection['hidden'] += 1
                continue
            attached = not placed or self._connected(box, placed)
            connected[identity] = attached
            if not attached and not self._floor_frontier(box, placed):
                rejection['detached'] += 1
                continue
            eligible.append(box)
        attached_levels = {round(b.pos[2], 7) for b in eligible if connected[(b.pos, b.dims)]}
        retained = [b for b in eligible if connected[(b.pos, b.dims)]
                    or round(b.pos[2], 7) not in attached_levels]
        rejection['floor_fallback_deferred'] = len(eligible)-len(retained)
        eligible = retained
        generated = time.perf_counter()
        selected = self._select(eligible, placed, profile, ordering)
        ended = time.perf_counter()
        features = {name: sorted({value for box in selected for key, value in self._families(box, placed)
                                 if key == name}) for name in ('yaw', 'height', 'frontier')}
        stats = dict(total_s=ended-began, generation_s=generated-began,
            selection_s=ended-generated, raw_tested=len(tried), raw_valid=raw_valid,
            frontier_valid=len(eligible), frontier_rejections=rejection,
            selected=len(selected), returned=len(selected), anchor_planes=len(intervals),
            adjacent_candidates=int(sum(self.contacts(b, placed)[0] > 1e-9 for b in selected)),
            ordering=ordering, diversity=features, candidate_limit=self.max_candidates,
            variant='exposed_face_frontier_v3', lowest_exposed_support_only=False,
            selected_support_levels_m=sorted({b.pos[2] for b in selected}),
            exact_contact_selected=int(sum(connected[(b.pos, b.dims)] for b in selected)),
            floor_frontier_fallback=bool(selected and not connected[(selected[0].pos, selected[0].dims)]),
            depth_step_m_used=False, dense_voxel_enumeration=False,
            support=f'Exact same-height >={100*self.min_support_coverage:g}% coverage + supported center',
            sphere='Inherited final TCP sphere only; not IK/path feasibility',
            scene_prefix=len(placed), future_arrivals_used=0, historical_xyz_used=False)
        sink = getattr(self, 'trace_sink', None)
        if sink is not None:
            data = dict(dims=dims, placed=placed, tried=tried, geometric=list(rows.values()),
                        eligible=eligible, selected=selected, stats=stats)
            if getattr(self, '_defer_trace', False):
                self._deferred_trace = data
            else:
                sink(data)
        return selected, stats
