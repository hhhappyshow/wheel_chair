"""Online contact-first layout only; no arm, future items, grid or dynamics.

ContactLayoutPrior keeps SparseCompactPrior's <=10 submitted candidates and
exact >=80% coplanar support + center predicate. Unequal top heights are never
merged. A geometric failure means this sparse search found no candidate.
"""
import time

import numpy as np

from compare import Box
from straight_place.sparse_prior import SparseCompactPrior


def _union_area(rectangles):
    xs = sorted({v for r in rectangles for v in (r[0], r[2])})
    area = 0.
    for left, right in zip(xs, xs[1:]):
        intervals = sorted((b, d) for a, b, c, d in rectangles if a < right and c > left)
        end = -float('inf')
        for bottom, top in intervals:
            area += (right-left)*max(0., top-max(bottom, end))
            end = max(end, top)
    return float(area)


class ContactLayoutPrior(SparseCompactPrior):
    def __init__(self, cfg, hull, *, directional_band_m=.10,
                 use_endpoint_sphere=False,layer_preference_band_m=0., **kwargs):
        super().__init__(cfg, hull, use_endpoint_sphere=use_endpoint_sphere, **kwargs)
        if directional_band_m <= 0:
            raise ValueError('directional_band_m must be positive')
        self.directional_band_m = float(directional_band_m)
        self._score_cache = {}
        self.layer_preference_band_m=float(layer_preference_band_m)

    def supported(self, box, placed):
        padding = getattr(self, 'physical_padding_xy_m', 0.)
        if not padding:
            return super().supported(box, placed)
        # Collision margins are not material that can support another box.
        def physical(b):
            return Box(b.id, (b.dims[0]-2*padding, b.dims[1]-2*padding, b.dims[2]),
                       (b.pos[0]+padding, b.pos[1]+padding, b.pos[2]))
        return super().supported(physical(box), [physical(b) for b in placed])

    def score(self, box, placed):
        """Exact face areas and bounding-envelope void, not a stability claim."""
        low = np.asarray(box.pos); high = low+box.dims
        neighbor = 0.; wall_faces = {}
        wall_planes = {(0, 1): ('rear', self.hull.length),
                       (1, 0): ('right', 0.), (1, 1): ('left', self.hull.width),
                       (2, 0): ('floor', 0.)}
        for axis in range(3):
            others = [i for i in range(3) if i != axis]
            face_area = float(np.prod(np.asarray(box.dims)[others]))
            for side in (0, 1):
                plane = (low, high)[side][axis]
                rectangles = []
                for other in placed:
                    olow = np.asarray(other.pos); ohigh = olow+other.dims
                    if abs(plane-(ohigh, olow)[side][axis]) > (self.neighbor_gap_m+1e-7 if axis<2 else 1e-7):
                        continue
                    a = np.maximum(low[others], olow[others])
                    b = np.minimum(high[others], ohigh[others])
                    if np.all(b > a):
                        rectangles.append((a[0], a[1], b[0], b[1]))
                neighbor += _union_area(rectangles)
                if (axis, side) in wall_planes:
                    name, location = wall_planes[(axis, side)]
                    if abs(plane-location) <= 1e-7:
                        wall_faces[name] = face_area
        surface = float(2*(box.dims[0]*box.dims[1]+box.dims[0]*box.dims[2]+box.dims[1]*box.dims[2]))
        volume = float(np.prod(box.dims))
        if placed:
            lows = np.asarray([b.pos for b in placed]); highs = lows+np.asarray([b.dims for b in placed])
            old_low, old_high = lows.min(0), highs.max(0)
            old_envelope = float(np.prod(old_high-old_low))
            envelope = float(np.prod(np.maximum(old_high, high)-np.minimum(old_low, low)))
        else:
            old_envelope, envelope = 0., volume
        occupied_volume = volume+sum(float(np.prod(b.dims)) for b in placed)
        wall = float(sum(wall_faces.values()))
        gap = max(0., envelope-occupied_volume)
        return dict(neighbor_contact_area_m2=neighbor, wall_contact_area_m2=wall,
            normalized_neighbor_contact=neighbor/surface, normalized_wall_contact=wall/surface,
            normalized_total_contact=(neighbor+wall)/surface, wall_faces_m2=wall_faces,
            envelope_growth_m3=envelope-old_envelope, envelope_growth_over_box_volume=(envelope-old_envelope)/volume,
            envelope_gap_m3=gap, envelope_gap_fraction=gap/envelope,
            gap_definition='Bounding-envelope void only, not full-hull free-space or insertion feasibility')

    def _support_layer(self, box, placed):
        height = box.pos[2]
        if self.layer_preference_band_m:
            anchors = []
            for z in sorted({0.} | {b.pos[2]+b.dims[2] for b in placed}):
                if not anchors or z-anchors[-1] > self.layer_preference_band_m:
                    anchors.append(z)
            height = next(i for i, z in enumerate(anchors)
                          if box.pos[2] <= z+self.layer_preference_band_m+1e-7)
        return height

    def _rank(self, box, placed, profile, ordering):
        identity = (box.pos, box.dims)
        if identity not in self._score_cache:
            self._score_cache[identity] = self.score(box, placed)
        score = self._score_cache[identity]
        band = self.directional_band_m
        height = self._support_layer(box, placed)
        if getattr(self, 'horizontal_row_first', False):
            rear = box.pos[0]+box.dims[0]
            anchor = self._rear_by_layer.get(height, self.hull.length)
            # Anchor bands to the available frontier, not an arbitrary world-grid origin.
            depth_band = int(np.floor(max(0., anchor-rear+1e-9)/band))
            directional = (height, depth_band, int(box.dims[0] > box.dims[1]),
                           int(np.floor((box.pos[1]+1e-9)/band)))
            return (*directional, -score['normalized_total_contact'], score['envelope_growth_m3'],
                    score['envelope_gap_fraction'], box.pos[1], -rear,
                    self.key(box, placed, profile))
        directional = (height, -int(np.floor((box.pos[0]+box.dims[0]+1e-9)/band)),
                       int(np.floor((box.pos[1]+1e-9)/band)))
        return (*directional, -score['normalized_total_contact'], score['envelope_growth_m3'],
                score['envelope_gap_fraction'], box.pos[1], -box.pos[0]-box.dims[0],
                self.key(box, placed, profile))

    def _select(self, rows, placed, profile, ordering):
        self._frontier_pool = list(rows)
        self._rear_by_layer = {}
        for box in self._frontier_pool:
            layer = self._support_layer(box, placed)
            self._rear_by_layer[layer] = max(self._rear_by_layer.get(layer, -np.inf),
                                              box.pos[0]+box.dims[0])
        return super()._select(self._frontier_pool, placed, profile, ordering)

    def _union_anchors(self, rows, placed):
        start = time.perf_counter()
        identities = {(b.pos, b.dims) for b in rows}
        self._union_tested = self._union_added = 0
        # Supplement per-box anchors with five anchors on each coplanar top
        # union. No bridging unequal heights or relaxing support coverage.
        seen_tops = []
        for support in placed:
            z = support.pos[2]+support.dims[2]
            if any(abs(z-old) <= 1e-7 for old in seen_tops):
                continue
            seen_tops.append(z)
            group = [b for b in placed if abs(b.pos[2]+b.dims[2]-z) <= 1e-7]
            if len(group) < 2:
                continue
            lower = np.asarray([b.pos[:2] for b in group]).min(0)
            upper = np.asarray([np.asarray(b.pos[:2])+b.dims[:2] for b in group]).max(0)
            for size in self._current_sizes:
                a, b = lower, upper-np.asarray(size[:2])
                for x, y in ((a[0], a[1]), (a[0], b[1]), (b[0], a[1]),
                             (b[0], b[1]), ((a[0]+b[0])/2, (a[1]+b[1])/2)):
                    y = max(float(y), self.lateral_clearance_m,
                            self.lateral_bounds[0] if self.lateral_bounds is not None else 0.)
                    interval = self.interval(size, y, z)
                    if interval is None:
                        continue
                    x = min(max(float(x), interval[0]), interval[1])
                    box = Box('current', size, (x, y, float(z)))
                    if getattr(self, '_defer_trace', False):
                        self._union_trace.append(box)
                    identity = (box.pos, box.dims)
                    if identity in identities:
                        continue
                    identities.add(identity); self._union_tested += 1
                    if self.valid(box, placed) and self._exposed(box, placed) and self._connected(box, placed):
                        rows.append(box); self._union_added += 1
        self._union_s = time.perf_counter()-start
        return rows

    def candidates(self, dims, placed, profile='depth', *, ordering=None):
        began = time.perf_counter()
        placed = list(placed)
        dims = tuple(float(v) for v in dims)
        self._current_sizes = sorted({dims, (dims[1], dims[0], dims[2])})
        self._score_cache = {}
        sink = getattr(self, 'trace_sink', None)
        self._defer_trace = sink is not None
        self._union_trace = []
        try:
            rows, stats = super().candidates(dims, placed, profile, ordering=ordering)
            pool = self._union_anchors(self._frontier_pool, placed)
        finally:
            self._defer_trace = False
        rows = super()._select(pool, placed, profile, self.ordering if ordering is None else ordering)
        scores = [self.score(b, placed) for b in rows]
        stats.update(variant='contact_layout_v2_deep_row', arm_planning=False,
            neighbor_gap_m=self.neighbor_gap_m,side_contact_score_is_proximity=bool(self.neighbor_gap_m),
            layer_preference_band_m=self.layer_preference_band_m,
            directional_order='lowest support -> deep rear-face band -> right-to-left band -> contact',
            directional_band_m=self.directional_band_m, exact_face_contact_tolerance_m=1e-7,
            raw_tested=stats['raw_tested']+self._union_tested,
            raw_valid=stats['raw_valid']+self._union_added,
            frontier_valid=stats['frontier_valid']+self._union_added,
            union_anchor_tested=self._union_tested, union_anchor_added=self._union_added,
            union_anchor_s=self._union_s, candidate_scores=scores,
            chosen_score=scores[0] if scores else None, selected=len(rows), returned=len(rows),
            selected_support_levels_m=sorted({b.pos[2] for b in rows}),
            adjacent_candidates=int(sum(self.contacts(b, placed)[0] > 1e-9 for b in rows)),
            exact_contact_selected=int(sum(not placed or self._connected(b, placed) for b in rows)),
            floor_frontier_fallback=bool(rows and placed and not self._connected(rows[0], placed)),
            diversity={name: sorted({value for box in rows for key, value in self._families(box, placed)
                                    if key == name}) for name in ('yaw', 'height', 'frontier')})
        stats['total_s'] = time.perf_counter()-began
        if getattr(self, 'horizontal_row_first', False):
            stats['directional_order'] = 'lowest support -> available rear-depth band -> horizontal -> right-to-left -> contact'
        if sink is not None:
            data = self._deferred_trace
            tried = set(data['tried']) | {(b.pos, b.dims) for b in self._union_trace}
            geometric = {(b.pos, b.dims): b for b in data['geometric']}
            geometric.update({(b.pos, b.dims): b for b in self._union_trace if self.valid(b, placed)})
            sink(dict(dims=dims, placed=placed, tried=tried, geometric=list(geometric.values()),
                      eligible=pool, selected=rows, stats=stats))
        return rows, stats


def cpu_comparison(cfg, hull, seeds=(73, 74, 75), count=12):
    """Paired current-only +/-8% size draws, no pre-generated future sequence."""
    reports = []
    for sphere in (False, True):
        for seed in seeds:
            rng = np.random.default_rng(seed)
            priors = {'sparse_greedy': SparseCompactPrior(cfg, hull, max_box_top_m=1., use_endpoint_sphere=sphere),
                      'contact': ContactLayoutPrior(cfg, hull, max_box_top_m=1., use_endpoint_sphere=sphere)}
            occupied = {name: [] for name in priors}
            metrics = {name: dict(accepted=0, rejected=0, total_s=0., submitted=0) for name in priors}
            for index in range(count):
                dims = np.array([.55, .40, .25])*rng.uniform(.92, 1.08, 3)
                for name, prior in priors.items():
                    rows, stats = prior.candidates(dims, occupied[name])
                    metrics[name]['total_s'] += stats['total_s']
                    metrics[name]['submitted'] += len(rows)
                    if rows:
                        box = rows[0]
                        occupied[name].append(Box(f'B{index+1}', box.dims, box.pos))
                        metrics[name]['accepted'] += 1
                    else:
                        metrics[name]['rejected'] += 1
            for name in priors:
                metrics[name]['mean_ms_per_arrival'] = metrics[name]['total_s']/count*1000
            reports.append(dict(seed=int(seed), arrivals=count, sphere=sphere, methods=metrics,
                                future_arrivals_used=0, scope='CPU sparse geometric layout only; no arm or dynamics'))
    return reports
