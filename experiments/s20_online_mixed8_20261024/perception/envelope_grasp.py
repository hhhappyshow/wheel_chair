"""CPU metric cloud fit and fixed-tool footprint placement. No simulator imports.

T_a_b maps b coordinates into a. Box origin is its volume center; box X is
the long top edge, Z points up. TCP +Z points into the contacted surface.
"""
import argparse
import itertools
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial import ConvexHull, cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial.transform import Rotation


def transform(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(
            element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return result


def stl_vertices(path):
    # STL's binary triangle record is specified, including its attribute word.
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], 'little')
    if len(data) == 84 + count * 50:
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)),
                          ('attribute', '<u2')])
        return np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
    return np.array([list(map(float, line.split()[1:])) for line in
                     data.decode('ascii').splitlines() if line.strip().startswith('vertex ')])


def load_tool(path):
    """URDF fixed subtree, or JSON components with vertices_tool/T_tcp_tool.

    JSON may also supply vertices_tcp directly, contact_vertices_tcp and a
    fixture_direction_tcp. No hidden flange/TCP coordinate substitution.
    """
    path = Path(path)
    if path.suffix == '.json':
        spec = json.loads(path.read_text())
        parts = []
        for component in spec['components']:
            pts = (np.array(component['vertices_tcp']) if 'vertices_tcp' in component else
                   transform(np.array(component['vertices_tool']),
                             np.array(component['T_tcp_tool'])))
            parts.append(dict(name=component['name'], vertices=pts))
        return parts, np.array(spec['contact_vertices_tcp']), np.array(spec['fixture_direction_tcp']), spec
    xml = ET.parse(path).getroot()
    frames = {'suction_panel': np.eye(4)}
    changed = True
    while changed:
        changed = False
        for joint in xml.findall('joint'):
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if joint.get('type') == 'fixed' and parent in frames and child not in frames:
                frames[child] = frames[parent] @ origin(joint.find('origin'))
                changed = True
    tcp_inv = np.linalg.inv(frames['suction_contact_frame'])
    parts, sources, contact = [], [], None
    for link in xml.findall('link'):
        name = link.get('name')
        if name not in frames or name in ('payload_box', 'suction_contact_frame'):
            continue
        elements = link.findall('collision') or link.findall('visual')
        for i, element in enumerate(elements):
            geometry = element.find('geometry')
            mesh, box = geometry.find('mesh'), geometry.find('box')
            if mesh is not None:
                filename = Path(mesh.get('filename'))
                if not filename.is_absolute():
                    filename = path.parent / filename
                pts = stl_vertices(filename) * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                label = filename.stem
                sources.append(str(filename))
            elif box is not None:
                size = np.fromstring(box.get('size'), sep=' ')
                pts = np.array(list(itertools.product(*zip(-size / 2, size / 2))))
                label = f'{name}_{i}'
            else:
                raise ValueError(f'Unsupported fixed-tool geometry: {name}')
            matrix = tcp_inv @ frames[name] @ origin(element.find('origin'))
            pts = transform(pts, matrix)
            parts.append(dict(name=f'{name}/{label}', vertices=pts))
            if name == 'suction_panel' and label == 'p04':
                contact = pts
    if contact is None:
        raise ValueError('p04 contact-side CAD not found; supply explicit tool JSON')
    # Sensor assembly side, rather than a nearly centered flange disc.
    fixture = np.mean([transform([[0, 0, 0]], tcp_inv @ frames[n])[0]
                       for n in ('camera_link', 'mid360_mount_frame')], axis=0)
    return parts, contact, fixture, dict(urdf=str(path.resolve()),
        mesh_sources=sources, T_panel_tcp=frames['suction_contact_frame'].tolist(),
        T_tcp_links={n: (tcp_inv @ t).tolist() for n, t in frames.items()},
        scope='fixed suction panel + adapter + camera + lidar; excludes arm and placeholder payload',
        lidar_geometry='URDF visual box; no collision element; included in envelope')


def fit_plane(points, normal_hint=(0, 0, 1), tolerance=.008):
    pts = np.asarray(points, float)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) < 20:
        raise ValueError('Too few finite plane points')
    sample = pts[::max(1, len(pts) // 12000)]
    rng = np.random.default_rng(24)
    best = np.zeros(len(sample), bool)
    hint = np.asarray(normal_hint, float)
    for _ in range(80):
        tri = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = np.linalg.norm(n)
        if norm < 1e-10:
            continue
        n /= norm
        if abs(n @ hint) < .94:
            continue
        mask = abs((sample - tri[0]) @ n) < tolerance
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < 20:
        raise ValueError('No supported horizontal plane')
    center = sample[best].mean(axis=0)
    _, _, vt = np.linalg.svd(sample[best] - center, full_matrices=False)
    n = vt[-1]
    if n @ hint < 0:
        n = -n
    return np.r_[n, -center @ n]


def fit_box(points, ground_plane, require_parallel=True):
    pts = np.asarray(points, float)
    pts = pts[np.isfinite(pts).all(axis=1)]
    ground = np.asarray(ground_plane, float)
    ground = ground / np.linalg.norm(ground[:3])
    if ground[2] < 0:
        ground = -ground
    plane = fit_plane(pts, ground[:3])
    surface = pts[abs(pts @ plane[:3] + plane[3]) < .008]
    surface = surface[::max(1, len(surface) // 16000)]
    edges = cKDTree(surface).query_pairs(.03, output_type='ndarray')
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(surface), len(surface)))
    _, labels = connected_components(graph, directed=False)
    surface = surface[labels == np.bincount(labels).argmax()]
    center = surface.mean(axis=0)
    _, _, vt = np.linalg.svd(surface - center, full_matrices=False)
    n = vt[-1]
    if n @ ground[:3] < 0:
        n = -n
    if require_parallel and n @ ground[:3] < .999:
        raise ValueError('This ground-standing cuboid model requires parallel top/floor')
    u = vt[0]
    v = np.cross(n, u)
    xy = np.c_[(surface - center) @ u, (surface - center) @ v]
    rect = cv2.minAreaRect(xy.astype('float32'))
    corners2 = cv2.boxPoints(rect).astype(float)
    corners = center + corners2[:, :1] * u + corners2[:, 1:] * v
    edges3 = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges3, axis=1)
    x = edges3[lengths.argmax()] / lengths.max()
    if x[0] < 0:
        x = -x
    top = corners.mean(axis=0)
    h = float(np.median(surface @ ground[:3] + ground[3]))
    dims = np.r_[sorted(rect[1], reverse=True), h]
    if np.any(dims <= 0):
        raise ValueError('Nonpositive measured box dimension')
    tf = np.eye(4)
    tf[:3, :3] = np.column_stack((x, np.cross(n, x), n))
    tf[:3, 3] = top - n * h / 2
    vertices = np.r_[corners, corners - n * h]
    return dict(dimensions_m=dims.tolist(), T_world_box=tf.tolist(),
                top_center_world_m=top.tolist(), top_corners_world_m=corners.tolist(),
                ground_plane=ground.tolist(), top_plane=plane.tolist(),
                top_floor_angle_deg=float(np.degrees(np.arccos(np.clip(n @ ground[:3], -1, 1)))),
                parallel_top_required=require_parallel,
                plane_rms_m=float(np.sqrt(np.mean(((surface-center) @ n)**2))),
                surface_points=len(surface), yaw_world_deg=float(np.degrees(np.arctan2(x[1], x[0]))),
                aabb_min_world_m=vertices.min(axis=0).tolist(),
                aabb_max_world_m=vertices.max(axis=0).tolist(),
                largest_face_up=bool(h <= min(dims[:2])),
                source='metric point-cloud RANSAC + connected surface + minimum-area rectangle',
                limitation='visible rectangle only; occluded extent cannot be recovered from this fit')


def polygon(points):
    pts = np.unique(np.asarray(points)[:, :2], axis=0)
    return pts[ConvexHull(pts).vertices]


def solve_grasp(fitted, parts, contact, fixture, center_world=None, angles=(0, 90, 180, 270), margin=.005,
                centered=True, preferred_fixture_direction_world=None):
    dims = np.array(fitted['dimensions_m'])
    box = np.array(fitted['T_world_box'])
    top = np.array(fitted['top_center_world_m'])
    down = Rotation.from_euler('x', np.pi).as_matrix()
    # p04 is square in this CAD. Choose its edge direction along which the
    # WHOLE fixed assembly spans furthest, preserving plate-edge parallelism.
    plate = cv2.boxPoints(cv2.minAreaRect((contact @ down.T)[:, :2].astype('float32')))
    edges = np.roll(plate, -1, axis=0) - plate
    unit_edges = edges / np.linalg.norm(edges, axis=1)[:, None]
    assembly_xy = (np.concatenate([p['vertices'] for p in parts]) @ down.T)[:, :2]
    axis = max(unit_edges, key=lambda a: np.ptp(assembly_xy @ a))
    if axis[0] < -1e-8 or abs(axis[0]) <= 1e-8 and axis[1] < 0:
        axis = -axis
    plate_yaw = np.arctan2(axis[1], axis[0])
    candidates = []
    for angle in angles:
        rotation = Rotation.from_euler('z', np.radians(angle) - plate_yaw).as_matrix() @ down
        projections = [p['vertices'] @ rotation.T for p in parts]
        all_xy = np.concatenate(projections)[:, :2]
        lo, hi = all_xy.min(axis=0), all_xy.max(axis=0)
        contact_xy = (contact @ rotation.T)[:, :2]
        plate_lower = -dims[:2]/2 + margin - contact_xy.min(axis=0)
        plate_upper = dims[:2]/2 - margin - contact_xy.max(axis=0)
        lower = np.maximum(-dims[:2]/2 + margin - lo, -dims[:2]/2 + margin)
        upper = np.minimum(dims[:2]/2 - margin - hi, dims[:2]/2 - margin)
        fits = bool(np.all(lower <= upper))
        # If oversized, center its conservative envelope for minimum max overhang.
        if centered:
            bias = np.zeros(2)
            fits = bool(np.all(lo >= -dims[:2]/2+margin) and np.all(hi <= dims[:2]/2-margin))
        else:
            bias = np.clip(-(lo+hi)/2, -dims[:2]/2+margin, dims[:2]/2-margin)
            if fits:
                bias = np.clip(bias, lower, upper)
            elif np.all(plate_lower <= plate_upper):
                bias = np.clip(bias, plate_lower, plate_upper)
        bounds = np.array([lo+bias, hi+bias])
        overflow = np.maximum(0, np.r_[-dims[:2]/2-bounds[0], bounds[1]-dims[:2]/2])
        world = np.eye(4)
        world[:3, :3] = box[:3, :3] @ rotation
        world[:3, 3] = top + box[:3, :2] @ bias
        side = world[:3, :3] @ fixture
        direction = np.zeros(3) if center_world is None else np.asarray(center_world)-world[:3, 3]
        direction -= box[:3, 2]*(direction @ box[:3, 2])
        side -= box[:3, 2]*(side @ box[:3, 2])
        inward = float(side @ direction/(np.linalg.norm(side)*np.linalg.norm(direction)+1e-12))
        preferred = np.zeros(3) if preferred_fixture_direction_world is None else np.asarray(preferred_fixture_direction_world, float)
        preferred = preferred-box[:3,2]*(preferred @ box[:3,2])
        preferred_score = float(side @ preferred/(np.linalg.norm(side)*np.linalg.norm(preferred)+1e-12))
        plate_xy = contact_xy+bias
        plate_fits = bool(np.all(plate_xy.min(axis=0) >= -dims[:2]/2+margin) and
                          np.all(plate_xy.max(axis=0) <= dims[:2]/2-margin))
        aligned = bool(angle % 180 == 0)
        candidates.append(dict(angle_deg=angle, long_axis_aligned=aligned,
            plate_edges_parallel_to_box=True, fixture_side_world=side.tolist(),
            preferred_fixture_direction_score=preferred_score,
            full_envelope_fits=fits, physically_inside=bool(overflow.max() <= 1e-9),
            containment_status=('fully_inside_with_margin' if fits else
                'cannot_fully_contain_partial_overhang' if overflow.max() > 1e-9 else 'inside_but_margin_unavailable'),
            contact_plate_fits=plate_fits, tcp_offset_box_xy_m=bias.tolist(),
            usable_tcp_offset_interval_m=([[0.,0.],[0.,0.]] if centered else np.c_[lower, upper].tolist()) if fits else None,
            contact_plate_offset_interval_m=([[0.,0.],[0.,0.]] if centered else np.c_[plate_lower, plate_upper].tolist()) if plate_fits else None,
            envelope_span_box_xy_m=(hi-lo).tolist(), envelope_bounds_box_xy_m=bounds.tolist(),
            overhang_minus_x_minus_y_plus_x_plus_y_m=overflow.tolist(),
            max_overhang_m=float(overflow.max()), inward_score=inward,
            T_world_tcp=world.tolist(), projected_parts_box_xy=[dict(name=p['name'],
                hull_xy_m=polygon(x[:, :2]+bias).tolist()) for p, x in zip(parts, projections)]))
    largest_face_up = bool(dims[2] <= min(dims[:2]))
    eligible = [c for c in candidates if (centered or c['long_axis_aligned']) and c['contact_plate_fits'] and largest_face_up]
    if not eligible:
        chosen = None
    else:
        chosen = min(eligible, key=lambda c: (not c['full_envelope_fits'],
                     round(c['max_overhang_m'], 6), -c['preferred_fixture_direction_score'], -c['inward_score']))
    return dict(selected=chosen, candidates=candidates, margin_m=margin,
                box_dimensions_m=dims.tolist(),
                centered=centered, mode='centered' if centered else 'optional_offset_historical',
                direction_scope='orientation preference only; container +X must be transformed to world by caller; no collision guarantee',
                largest_face_up=largest_face_up,
                assembly_long_axis_in_down_tcp_xy=axis.tolist(),
                contact_ready=False,
                limitation='footprint geometry only; requires complete observed top, seal, IK and path validation; oversized assembly remains partial overhang')


def plot_result(fitted, result, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    rows = result['candidates']
    fig, axes = plt.subplots(1, len(rows), figsize=(5*len(rows), 5), squeeze=False)
    for ax, row in zip(axes[0], rows):
        long, short = fitted['dimensions_m'][:2]
        ax.add_patch(Rectangle((-long/2, -short/2), long, short, fill=False, edgecolor='black', linewidth=2, label='observed box top'))
        for i, part in enumerate(row['projected_parts_box_xy']):
            ax.add_patch(Polygon(part['hull_xy_m'], facecolor=plt.cm.tab10(i % 10),
                                 alpha=.28, edgecolor=plt.cm.tab10(i % 10), label=part['name']))
        ax.plot(*row['tcp_offset_box_xy_m'], 'kx', label='TCP')
        ax.set_title(f"{row['angle_deg']} deg; full fit={row['full_envelope_fits']}\nmax overhang {1000*row['max_overhang_m']:.1f} mm")
        ax.set_xlabel('box long axis [m]'); ax.set_ylabel('box short axis [m]')
        ax.autoscale_view(); ax.set_aspect('equal'); ax.grid(alpha=.2)
    axes[0, -1].legend(fontsize=6, loc='upper left', bbox_to_anchor=(1.02, 1))
    fig.tight_layout(); fig.savefig(output, dpi=150); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--observation', type=Path, help='saved NPZ; points_world, selected, ground_plane')
    inputs.add_argument('--fit', type=Path, help='measured dimensions_m/T_world_box/top_center_world_m JSON')
    parser.add_argument('--tool', type=Path, required=True, help='flattened URDF or explicit CAD/TCP JSON')
    parser.add_argument('--container-center', nargs=3, type=float)
    parser.add_argument('--angles', nargs='+', type=int, choices=[0, 90, 180, 270], default=[0, 90, 180, 270])
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--centered', dest='centered', action='store_true', help='default: TCP exactly at fitted top center')
    modes.add_argument('--allow-offset', dest='centered', action='store_false', help='explicit optional historical offset mode')
    parser.set_defaults(centered=True)
    parser.add_argument('--preferred-fixture-direction-world', nargs=3, type=float)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['MPLCONFIGDIR'] = str(args.output/'mpl_config')
    if args.observation:
        data = np.load(args.observation)
        selected = data['selected'].astype(bool)
        fitted = fit_box(data['points_world'][selected][::2], data['ground_plane'])
        fitted['selection_source'] = 'saved geometric connected-component selected mask, NOT YOLO'
        fitted['image_border_clipped'] = bool(selected[0].any() or selected[-1].any() or selected[:, 0].any() or selected[:, -1].any())
    else:
        fitted = json.loads(args.fit.read_text())
    parts, contact, fixture, tool = load_tool(args.tool)
    result = solve_grasp(fitted, parts, contact, fixture, args.container_center, args.angles,
                        centered=args.centered, preferred_fixture_direction_world=args.preferred_fixture_direction_world)
    result.update(fitted=fitted, tool=tool, input=str(args.observation or args.fit),
                  hardware_motion=False, gpu_used=False)
    (args.output/'result.json').write_text(json.dumps(result, indent=2))
    plot_result(fitted, result, args.output/'coverage.png')
    print(json.dumps(dict(output=str(args.output), full_fits=bool(result['selected'] and result['selected']['full_envelope_fits']),
                          selected_angle=None if result['selected'] is None else result['selected']['angle_deg'])))


if __name__ == '__main__':
    main()
