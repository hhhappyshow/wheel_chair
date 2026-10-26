"""Inspect local vintage OBJ on CPU; --capture explicitly enables Newton GPU RGB-D.

Minimal isolated floor/mesh/camera, without robot or container occluders. Original
OBJ UVs and PNG are passed to Newton Mesh, not painted over a GT pixel mask.
The mesh scale/yaw instantiate the scene only; YOLO receives RGB and metric depth.
"""
import argparse
import json
import os
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import cv2
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
import trimesh


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh', type=Path, default=root/'reproductions/ludan_greedy_20261023/src/luggage_gazebo/models/suitcase_vintage/meshes/suitcase.obj')
    parser.add_argument('--texture', type=Path, help='override embedded OBJ material PNG')
    parser.add_argument('--size', type=int, choices=(20,24), default=20)
    parser.add_argument('--seed', type=int, default=24)
    parser.add_argument('--yaw-deg', type=float)
    parser.add_argument('--camera-reference', type=Path, default=root/'experiments/s20_greedy_async_20261023/mixed8_horizontal_observed_attempt2/perception/B1/observed_box.json')
    parser.add_argument('--capture', action='store_true', help='explicit GPU action; omit for CPU asset preparation')
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--run-yolo', action='store_true', help='after capture, run YOLO on CPU')
    parser.add_argument('--weights', type=Path, default=Path('/home/ubuntu/yolo11n-seg.pt'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['WARP_CACHE_PATH'] = str(args.output/'warp_cache')
    os.environ['YOLO_CONFIG_DIR'] = str(args.output/'yolo_config')
    os.environ['MPLCONFIGDIR'] = str(args.output/'mpl_config')
    os.environ['XDG_CACHE_HOME'] = str(args.output/'cache')
    mesh = trimesh.load(args.mesh, force='mesh', process=False)
    uv = np.asarray(mesh.visual.uv)
    if uv.shape != (len(mesh.vertices),2):
        raise ValueError('OBJ has no usable per-vertex UVs')
    # This OBJ's material maps suitcase.png; use the loaded material image so
    # UV seams and texel data survive the import instead of falling back to STL.
    image = Image.open(args.texture) if args.texture else mesh.visual.material.image
    texture = np.asarray(image.convert('RGBA'))
    dims = np.array([.55,.40,.25] if args.size == 20 else [.65,.45,.30])
    vertices = ((mesh.vertices-mesh.bounds.mean(axis=0))/mesh.extents)*dims
    yaw = args.yaw_deg if args.yaw_deg is not None else float(np.random.default_rng(args.seed).uniform(-180,180))
    report = dict(mesh=str(args.mesh.resolve()), source_bounds=mesh.bounds.tolist(),
       vertex_count=len(vertices), triangle_count=len(mesh.faces), uv_shape=list(uv.shape),
       texture_shape=list(texture.shape), texture_rgb_std=float(texture[:,:,:3].std()),
       texture_source=str(args.texture) if args.texture else 'OBJ material.mtl -> suitcase.png',
       normalized_to_dimensions_m=dims.tolist(), size_inches_proxy=args.size, seed=args.seed, yaw_deg=yaw,
       geometry_scope='nonuniform scale of local laid-flat vintage asset; not a measured 20/24 suitcase',
       camera_reference=str(args.camera_reference), gpu_capture_executed=False,
       status='CPU_ASSET_READY_GPU_CAPTURE_PENDING', gt_mask_used=False,
       scene_limitation='isolated floor and suitcase; no S20/ULD/fixture occlusion, no dynamics')
    if args.capture:
        # No Warp/Newton import is reached during the CPU-only inspection.
        import warp as wp
        import newton
        from newton.sensors import SensorTiledCamera
        from newton._src.sensors.warp_raytrace.types import RenderConfig
        wp.config.kernel_cache_dir = str(args.output/'warp_cache')
        wp.set_device(args.device)
        builder = newton.ModelBuilder()
        builder.add_ground_plane(color=(.40,.43,.45))
        native_mesh = newton.Mesh(vertices, mesh.faces.reshape(-1),
             uvs=uv, texture=texture, compute_inertia=False, color=(1.,1.,1.))
        builder.add_shape_mesh(body=-1, mesh=native_mesh,
             xform=wp.transform(p=wp.vec3(0,1.6,float(dims[2]/2)),
                                q=wp.quat(*Rotation.from_euler('z',yaw,degrees=True).as_quat())),
             label='vintage_textured_mesh')
        model = builder.finalize(device=args.device)
        state = model.state()
        model.bvh_build_shapes(state)
        camera = SensorTiledCamera(model, config=RenderConfig(enable_textures=True, enable_shadows=False))
        camera.utils.create_default_light(enable_shadows=False)
        rays = camera.utils.compute_pinhole_camera_rays(640,360,2*np.arctan(180/322.8994))
        depth_out = camera.utils.create_depth_image_output(640,360)
        color_out = camera.utils.create_color_image_output(640,360)
        reference = json.loads(args.camera_reference.read_text())
        position = np.array(reference['camera_gl_origin'])
        quat = np.array(reference['camera_gl_xyzw'])
        pose = wp.array([[wp.transformf(wp.vec3(*position),wp.quat(*quat))]],dtype=wp.transformf)
        camera.update(state,pose,rays,depth_image=depth_out,color_image=color_out)
        depth = depth_out.numpy()[0,0]
        rgba = camera.utils.to_rgba_from_color(color_out).numpy().reshape(360,640,4)
        bgr = rgba[:,:,:3][:,:,::-1]
        cloud = Rotation.from_quat(quat).apply((rays.numpy()[0,:,:,1]*depth[:,:,None]).reshape(-1,3))+position
        cloud = cloud.reshape(360,640,3)
        cloud[~np.isfinite(depth)|(depth<=0)] = np.nan
        cv2.imwrite(str(args.output/'camera_rgb.png'),bgr)
        np.savez_compressed(args.output/'camera_raw.npz',points_world=cloud,depth_range_m=depth)
        report.update(status='TEXTURED_NEWTON_CAPTURE_COMPLETED', gpu_capture_executed=True,
             camera_gl_origin=position.tolist(),camera_gl_xyzw=quat.tolist(),
             rgb_std=float(bgr.std()),valid_depth_pixels=int(np.isfinite(cloud).all(axis=2).sum()))
        if args.run_yolo:
            from ultralytics import YOLO
            from yolo_sim_perception import infer
            result, mask, fitted = infer(YOLO(str(args.weights)),bgr,cloud,device='cpu')
            cv2.imwrite(str(args.output/'yolo_overlay.png'),infer.last_overlay)
            if mask is not None:
                cv2.imwrite(str(args.output/'yolo_selection.png'),mask.astype('uint8')*255)
            (args.output/'yolo_result.json').write_text(json.dumps(result,indent=2))
            if fitted:
                from grasp_interface import EnvelopeGraspPlanner
                planner = EnvelopeGraspPlanner(root/'experiments/s20_greedy_async_20261023/v2_s20_model_h100/s20_current.urdf')
                grasp = planner.plan(fitted, centered=True)
                (args.output/'centered_grasp.json').write_text(json.dumps(grasp,indent=2))
            report['yolo_status'] = result['status']
    (args.output/'capture_status.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))


if __name__ == '__main__':
    main()
