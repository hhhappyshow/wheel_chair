"""Exact planning support, stdlib only; no FK correction or position mutation.

Box objects provide .id/.pos/.dims; hull provides .floor_supports(box).
Use committed PLANNING support surfaces as input. A frozen FK pose 5mm above
its support is not contact: reconcile that representation explicitly upstream.
"""
HEIGHT_EPS_M = 1e-7
MIN_COVERAGE = .8


def support_details(box, placed, hull, min_coverage=MIN_COVERAGE):
    """Union footprint coverage and center-in-contact-union (uniform box COM).

    No height relaxation, footprint expansion, snapping, friction or stability
    inference. HEIGHT_EPS_M is numerical equality, not a release-gap allowance.
    """
    x, y, z = box.pos
    dx, dy, _ = box.dims
    if dx <= 0 or dy <= 0:
        raise ValueError("Positive box footprint required")
    if hull.floor_supports(box):
        return dict(supported=True, floor=True, coverage=1., contact_area_m2=dx*dy,
                    center_supported=True, supporter_ids=[], height_tolerance_m=HEIGHT_EPS_M)
    rectangles = []
    ids = []
    for other in placed:
        if other.id == box.id or abs(other.pos[2]+other.dims[2]-z) > HEIGHT_EPS_M:
            continue
        x0, y0 = max(x, other.pos[0]), max(y, other.pos[1])
        x1, y1 = min(x+dx, other.pos[0]+other.dims[0]), min(y+dy, other.pos[1]+other.dims[1])
        if x1 > x0 and y1 > y0:
            rectangles.append((x0, y0, x1, y1))
            ids.append(other.id)
    xs = sorted({v for r in rectangles for v in (r[0], r[2])})
    ys = sorted({v for r in rectangles for v in (r[1], r[3])})
    area = 0.
    for xa, xb in zip(xs, xs[1:]):
        for ya, yb in zip(ys, ys[1:]):
            cx, cy = (xa+xb)/2, (ya+yb)/2
            if any(a <= cx <= c and b <= cy <= d for a, b, c, d in rectangles):
                area += (xb-xa)*(yb-ya)
    cx, cy = x+dx/2, y+dy/2
    center = any(a-HEIGHT_EPS_M <= cx <= c+HEIGHT_EPS_M
                 and b-HEIGHT_EPS_M <= cy <= d+HEIGHT_EPS_M for a, b, c, d in rectangles)
    coverage = min(1., area/(dx*dy))
    return dict(supported=coverage >= min_coverage-1e-10 and center,
                floor=False, coverage=coverage, contact_area_m2=area,
                center_supported=center, supporter_ids=ids, height_tolerance_m=HEIGHT_EPS_M)


def supported(box, placed, hull):
    """Drop-in predicate: supported(candidate, committed_planning_boxes, hull)."""
    return support_details(box, placed, hull)["supported"]
