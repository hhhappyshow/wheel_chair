"""YOLO RGB detection/segmentation -> metric depth fit -> tool envelope.

Default: saved RGB + aligned points_world NPZ, CPU inference. --render is an
explicit GPU-only future experiment using the existing Newton camera/assets.
No saved selected mask, object IDs, true box dimensions or poses enter fitting.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from envelope_grasp import fit_plane, fit_box, load_tool, solve_grasp, plot_result


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2))


def infer(model, rgb, cloud, confidence=.25, device='cpu', locked_center_world=None,
          workspace_bounds_world_m=None, class_agnostic=False, require_parallel_top=True):
    if device == 'cpu':
        import torch
        from ultralytics.utils import torch_utils
        # select_device otherwise resets Torch to Ultralytics' host default.
        torch_utils.NUM_THREADS = int(os.environ.get('OMP_NUM_THREADS', '2'))
        torch.set_num_threads(torch_utils.NUM_THREADS)
    prediction = model.predict(rgb, device=device, conf=confidence, verbose=False,
                               retina_masks=True, save=False)[0]
    classes = [i for i, name in model.names.items() if name.lower() in ('suitcase', 'luggage')]
    if not classes and not class_agnostic:
        raise ValueError(f'No suitcase/luggage class in weight metadata: {model.names}')
    detections = [i for i, box in enumerate(prediction.boxes) if class_agnostic or int(box.cls.item()) in classes]
    record = dict(backend='ultralytics YOLO', classes=model.names,
                  gt_mask_used=False, saved_geometric_mask_used=False,
                  luggage_detections=len(detections), device=device,
                  inference_executed=True, speed_ms=prediction.speed,
                  raw_detections=[dict(class_id=int(b.cls.item()), class_name=model.names[int(b.cls.item())],
                    confidence=float(b.conf.item()), xyxy=b.xyxy[0].cpu().numpy().tolist()) for b in prediction.boxes])
    record.update(target_class_filter='any_detected_class' if class_agnostic else 'suitcase_or_luggage',
        accepted_target_detections=len(detections),
        luggage_detections=sum(int(box.cls.item()) in classes for box in prediction.boxes))
    infer.last_overlay = prediction.plot()
    if not detections:
        return dict(record, status='NO_YOLO_TARGET_DETECTION' if class_agnostic else 'NO_YOLO_LUGGAGE_DETECTION', contact_ready=False), None, None
    geometry_began=time.perf_counter()
    def detection_centroid(i):
        if prediction.masks is not None:
            region = prediction.masks.data[i].cpu().numpy() > .5
        else:
            region = np.zeros(cloud.shape[:2],bool)
            x0,y0,x1,y1 = prediction.boxes[i].xyxy[0].cpu().numpy().astype(int)
            region[max(0,y0):min(region.shape[0],y1),max(0,x0):min(region.shape[1],x1)] = True
        pts = cloud[region & np.isfinite(cloud).all(axis=2)]
        return np.median(pts,axis=0) if len(pts) else np.full(3,np.nan)
    centroids = {i:detection_centroid(i) for i in detections}
    if locked_center_world is None:
        index = max(detections, key=lambda i: float(prediction.boxes[i].conf.item()))
    else:
        usable = [i for i in detections if np.isfinite(centroids[i]).all()]
        if not usable:
            return dict(record,status='YOLO_DETECTED_NO_VALID_DEPTH_FOR_LOCK',contact_ready=False),None,None
        index = min(usable,key=lambda i:np.linalg.norm(centroids[i]-np.asarray(locked_center_world)))
    record.update(target_selection='max_confidence' if locked_center_world is None else 'nearest_YOLO_mask_depth_centroid_to_world_lock',
                  selected_depth_centroid_world_m=centroids[index].tolist(),
                  lock_distance_m=None if locked_center_world is None else float(np.linalg.norm(centroids[index]-np.asarray(locked_center_world))),
                  simulator_body_identity_used=False)
    bbox = prediction.boxes[index].xyxy[0].cpu().numpy()
    record.update(selected_class_id=int(prediction.boxes[index].cls.item()),
        selected_class_name=model.names[int(prediction.boxes[index].cls.item())])
    if prediction.masks is not None:
        mask = prediction.masks.data[index].cpu().numpy() > .5
        if mask.shape != cloud.shape[:2]:
            raise ValueError('YOLO mask and registered depth resolution differ')
        source = 'YOLO instance mask; depth RANSAC top plane'
    else:
        mask = np.zeros(cloud.shape[:2], bool)
        x0, y0, x1, y1 = bbox.astype(int)
        mask[max(0, y0):min(mask.shape[0], y1), max(0, x0):min(mask.shape[1], x1)] = True
        source = 'YOLO bounding box ROI + geometric depth plane, NOT YOLO segmentation'
    finite = np.isfinite(cloud).all(axis=2) & (np.linalg.norm(cloud, axis=2) > .01)
    # Known pickup workspace is a prior, never the true object's bounding box.
    bounds = np.array(workspace_bounds_world_m if workspace_bounds_world_m is not None else [[-2,2],[-1,3.5],[-.03,1.2]])
    workspace = finite & ((cloud>=bounds[:,0]) & (cloud<=bounds[:,1])).all(axis=2)
    record['known_floor_workspace_bounds_world_m'] = bounds.tolist()
    floor = cloud[workspace & ~mask]
    if len(floor) < 30:
        return dict(record, status='ADJUST_VIEW_REQUIRED', reason='insufficient visible ground', contact_ready=False), mask, None
    floor = floor[floor[:, 2] <= np.quantile(floor[:, 2], .35)]
    ground = fit_plane(floor)
    elevated = cloud @ ground[:3] + ground[3] > .07
    selected = mask & workspace & elevated
    fitted = fit_box(cloud[selected][::2], ground, require_parallel=require_parallel_top)
    fitted['selection_source'] = source
    # Completeness needs surrounding visible depth and supported rectangle
    # edges, not simply an interior image bbox or a successful plane fit.
    top = np.array(fitted['top_plane'])
    surface = selected & (abs(cloud @ top[:3] + top[3]) < .008)
    ring = cv2.dilate(surface.astype('uint8'), np.ones((9, 9), 'uint8')).astype(bool) & ~surface
    ring_valid = ring & finite & workspace
    lower = (cloud @ ground[:3] + ground[3]) < fitted['dimensions_m'][2] - .025
    exterior_fraction = float((ring_valid & lower).sum()/max(1, ring.sum()))
    tf = np.array(fitted['T_world_box'])
    xy = (cloud[surface]-np.array(fitted['top_center_world_m'])) @ tf[:3, :2]
    half = np.array(fitted['dimensions_m'][:2])/2
    grid = np.floor((xy+half)/(2*half)*30).astype(int).clip(0, 29)
    occupancy = np.zeros((30, 30), bool)
    occupancy[grid[:, 0], grid[:, 1]] = True
    edge_support = [float(v.mean()) for v in (occupancy[:2].any(axis=0), occupancy[-2:].any(axis=0),
                    occupancy[:, :2].any(axis=1), occupancy[:, -2:].any(axis=1))]
    clipped = bool(mask[:3].any() or mask[-3:].any() or mask[:, :3].any() or mask[:, -3:].any())
    reasons = ['mask_intersects_image_border'] if clipped else []
    record.update(status='ADJUST_VIEW_REQUIRED' if clipped else 'SURFACE_FITTED_COMPLETENESS_UNCONFIRMED',
                  detection_confidence=float(prediction.boxes[index].conf.item()),
                  detection_bbox_xyxy=bbox.tolist(), selection_source=source,
                  fitted=fitted, complete_top_confirmed=False, contact_ready=False,
                  observation_rejection_reasons=reasons,
                  completeness=dict(image_border_clipped=clipped, surface_grid_coverage=float(occupancy.mean()),
                                    four_edge_support=edge_support, visible_lower_exterior_fraction=exterior_fraction,
                                    limitation='reported metrics only; no 95/90 percent completion gate; does not certify full top'),
                  next_observation=None if not clipped else dict(raise_camera_m=.20,
                    center_on_world_m=fitted['top_center_world_m'], contact_before_reobserve=False))
    record['geometry_fit_wall_s']=time.perf_counter()-geometry_began
    return record, mask, fitted


def render_capture(runtime, output):
    import newton
    import warp as wp
    camera = runtime.state.body_q.numpy()[runtime.camera]
    link_rotation = Rotation.from_quat(camera[3:])
    gl = Rotation.from_matrix([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    rotation = link_rotation * gl
    position = camera[:3]+link_rotation.apply([.025, 0, 0])
    if getattr(runtime,'camera_driver_transform',None) is not None:
        driver=runtime.camera_driver_transform
        rotation=link_rotation*Rotation.from_matrix(driver[:3,:3])*Rotation.from_euler('x',np.pi)
        position=camera[:3]+link_rotation.apply(driver[:3,3])
    pose = wp.array([[wp.transformf(wp.vec3(*position), wp.quat(*rotation.as_quat()))]], dtype=wp.transformf)
    flags=None
    if getattr(runtime,'camera_driver_transform',None) is not None:
        # The optical center is inside the opaque CAD housing. Exclude only
        # that housing from its own image, not from physical collision checks.
        flags=runtime.model.shape_flags.numpy().copy();image_flags=flags.copy()
        image_flags[runtime.model.shape_body.numpy()==runtime.camera]&=~int(newton.ShapeFlags.VISIBLE)
        runtime.model.shape_flags.assign(image_flags);runtime.model.bvh_build_shapes(runtime.state)
    try:
        runtime.sensor.update(runtime.state, pose, runtime.rays,
                              depth_image=runtime.depth, color_image=runtime.color)
    finally:
        if flags is not None:
            runtime.model.shape_flags.assign(flags);runtime.model.bvh_build_shapes(runtime.state)
    depth = runtime.depth.numpy()[0, 0]
    rgba = runtime.sensor.utils.to_rgba_from_color(runtime.color).numpy().reshape(360, 640, 4)
    bgr = rgba[:, :, :3][:, :, ::-1]
    rays = runtime.rays.numpy()[0, :, :, 1]
    points = rotation.apply((rays*depth[:, :, None]).reshape(-1, 3))+position
    points = points.reshape(360, 640, 3)
    points[~np.isfinite(depth) | (depth <= 0)] = np.nan
    cv2.imwrite(str(output/'camera_rgb.png'), bgr)
    np.savez_compressed(output/'camera_raw.npz', points_world=points, depth_range_m=depth)
    return bgr, points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--rgb', type=Path)
    parser.add_argument('--observation', type=Path)
    parser.add_argument('--tool', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--conf', type=float, default=.25)
    parser.add_argument('--container-center', nargs=3, type=float)
    parser.add_argument('--render', action='store_true', help='explicit GPU Newton capture; not executed by CPU audit')
    parser.add_argument('--size', type=int, choices=(20, 24), default=20)
    parser.add_argument('--seed', type=int, default=24)
    parser.add_argument('--yaw-deg', type=float, help='omit for seeded random yaw in [-180,180]')
    parser.add_argument('--max-views', type=int, default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('YOLO_CONFIG_DIR', str(args.output/'yolo_config'))
    os.environ.setdefault('MPLCONFIGDIR', str(args.output/'mpl_config'))
    os.environ.setdefault('WARP_CACHE_PATH', str(args.output/'warp_cache'))
    os.environ.setdefault('XDG_CACHE_HOME', str(args.output/'cache'))
    if args.device == 'cpu' and not args.render:
        os.environ['CUDA_VISIBLE_DEVICES'] = ''
    report = dict(weights=str(args.weights.resolve()), hardware_motion=False,
                  simulation_render_requested=args.render, gt_mask_used=False,
                  saved_geometric_mask_used=False)
    try:
        if not args.weights.is_file():
            raise FileNotFoundError('Explicit local weights required; no automatic downloads')
        from ultralytics import YOLO
        import ultralytics
        import torch
        torch.set_num_threads(int(os.environ.get('OMP_NUM_THREADS', '2')))
        model = YOLO(str(args.weights))
        report.update(ultralytics_version=ultralytics.__version__, torch_version=torch.__version__,
                      cpu_threads=torch.get_num_threads(), python=sys.executable)
        runtime = None
        if args.render:
            root = Path(__file__).resolve().parents[3]
            sys.path.insert(0, str(root/'experiments/s20_online_mixed8_20261024'))
            from observed_cycle import Runtime
            dims = [.55, .4, .25] if args.size == 20 else [.65, .45, .30]
            yaw = args.yaw_deg if args.yaw_deg is not None else np.random.default_rng(args.seed).uniform(-180, 180)
            runtime = Runtime(args.output, False, catalog=[dims])
            runtime.fixed[runtime.boxes[0]] = np.r_[[0, 1.6, dims[2]/2], Rotation.from_euler('z', yaw, degrees=True).as_quat()]
            runtime.frame(runtime.q, 'YOLO_OBSERVE')
            runtime.model.bvh_build_shapes(runtime.state)
            report['simulation_instantiation_only'] = dict(proxy_size_inches=args.size, dimensions_m=dims, yaw_deg=yaw,
                asset='Newton colored cuboid proxy, not textured luggage; domain mismatch for COCO suitcase', seed=args.seed)
        elif not args.rgb or not args.observation:
            raise ValueError('Saved mode requires --rgb and --observation')
        parts, contact, fixture, tool = load_tool(args.tool)
        report['tool'] = tool
        report['views'] = []
        for view in range(args.max_views if runtime else 1):
            out = args.output/f'view{view}'
            out.mkdir(exist_ok=True)
            start = time.perf_counter()
            if runtime:
                rgb, cloud = render_capture(runtime, out)
            else:
                rgb = cv2.imread(str(args.rgb))
                cloud = np.load(args.observation)['points_world']
                if rgb is None or rgb.shape[:2] != cloud.shape[:2]:
                    raise ValueError('Aligned RGB and points_world HxWx3 required')
            record, mask, fitted = infer(model, rgb, cloud, args.conf, args.device)
            cv2.imwrite(str(out/'yolo_overlay.png'), infer.last_overlay)
            record['wall_s'] = time.perf_counter()-start
            report['views'].append(record)
            if mask is not None:
                cv2.imwrite(str(out/'yolo_selection.png'), mask.astype('uint8')*255)
            if fitted:
                grasp = solve_grasp(fitted, parts, contact, fixture, args.container_center)
                write_json(out/'envelope.json', grasp)
                plot_result(fitted, grasp, out/'coverage.png')
                record['envelope_full_fit'] = bool(grasp['selected'] and grasp['selected']['full_envelope_fits'])
            write_json(out/'result.json', record)
            if record['status'] == 'SURFACE_FITTED_COMPLETENESS_UNCONFIRMED' or not runtime:
                break
            # Empty-tool observation only; this script never contacts or attaches.
            tcp = runtime.state.body_q.numpy()[runtime.tcp]
            delta = np.array([0., 0., .20])
            if fitted:
                camera = runtime.state.body_q.numpy()[runtime.camera]
                delta[:2] = np.clip(np.array(fitted['top_center_world_m'])[:2]-camera[:2], -.10, .10)
            if view+1 < args.max_views:
                runtime.move([(tcp[:3]+delta, tcp[3:])], 'YOLO_ADJUST_VIEW', 2.)
                runtime.model.bvh_build_shapes(runtime.state)
        report['status'] = report['views'][-1]['status']
    except Exception as exc:
        report.update(status='BLOCKED', error=f'{type(exc).__name__}: {exc}',
                      contact_ready=False, fallback_used=False)
    write_json(args.output/'result.json', report)
    print(json.dumps(dict(status=report['status'], result=str(args.output/'result.json'))))
    return 2 if report['status'] == 'BLOCKED' else 0


if __name__ == '__main__':
    sys.exit(main())
