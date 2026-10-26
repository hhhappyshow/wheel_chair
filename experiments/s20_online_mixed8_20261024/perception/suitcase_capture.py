"""Native suitcase RGB/depth with unchanged cuboid CAD/FCL collision proxies.

Contract: make_runtime(output, catalog) -> current observed_cycle.Runtime.
Caller owns runtime.frame(q, stage), runtime.fixed body poses, and captures.
No simulator identity/dimensions from this module enter the YOLO estimator.
"""
import json
import os
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[3]
ASSET = ROOT/'reproductions/ludan_greedy_20261023/src/luggage_gazebo/models/suitcase_vintage/meshes/suitcase.obj'


def make_runtime(output, catalog, site_urdf=None):
    """catalog is scene-generation sizes only, [length,width,height] metres.

    Builds original invisible collision boxes and textured visible meshes on
    the SAME bodies before finalize. Newton sensor depth raycasts the visible
    mesh; FCL sees only the original box shapes. Original make_model restored
    immediately after this local Runtime construction. No placement backend.
    """
    output = Path(output)
    output.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('WARP_CACHE_PATH',str(output/'warp_cache'))
    os.environ.setdefault('PYOPENGL_USE_ACCELERATE','0')
    for path in (ROOT/'experiments/s20_greedy_async_20261023',ROOT/'experiments/s20_online_mixed8_20261024'):
        sys.path.insert(0,str(path))
    import observed_cycle
    import copy
    import importlib.util
    import xml.etree.ElementTree as ET
    import newton
    import warp as wp
    from roboticarm_newton import add_cell_scene
    import roboticarm_newton as native
    from newton.sensors import SensorTiledCamera
    from newton._src.sensors.warp_raytrace.types import RenderConfig
    asset = trimesh.load(ASSET,force='mesh',process=False)
    texture = np.asarray(asset.visual.material.image.convert('RGBA'))
    uv = np.asarray(asset.visual.uv)
    catalog = [list(map(float,d)) for d in catalog]
    generation = []
    def factory(sample,source_root,scene_config_path,container_y_override=None,*,show_robot=True):
        builder = newton.ModelBuilder()
        builder.default_shape_cfg.mu = .7
        builder.default_joint_cfg.target_ke = 6000.
        builder.default_joint_cfg.target_kd = 120.
        config = add_cell_scene(builder,source_root,show_robot=show_robot,
            scene_config_path=scene_config_path,container_y_override=container_y_override)
        ids = {}
        for step in sample['steps']:
            item = step['incoming']; label = item['id']; dims = np.array(item['dims'])
            body = builder.add_body(xform=wp.transform(p=wp.vec3(100,100,100)),label=label)
            proxy = builder.default_shape_cfg.copy(); proxy.is_visible = False
            builder.add_shape_box(body,hx=float(dims[0]/2),hy=float(dims[1]/2),hz=float(dims[2]/2),cfg=proxy,label=label+'_collision_box')
            visual = builder.default_shape_cfg.copy()
            visual.has_shape_collision = False; visual.has_particle_collision = False; visual.density = 0.
            verts = (asset.vertices-asset.bounds.mean(axis=0))/asset.extents*dims
            mesh = newton.Mesh(verts,asset.faces.reshape(-1),uvs=uv,texture=texture,
                               color=(1.,1.,1.),compute_inertia=False)
            builder.add_shape_mesh(body,mesh=mesh,cfg=visual,label=label+'_suitcase_visual')
            ids[label] = body
            generation.append(dict(id=label,dimensions_m=dims.tolist(),asset=str(ASSET),body_index=body))
        model = builder.finalize()
        flags = model.shape_flags.numpy(); bodies = model.shape_body.numpy()
        for i,b in enumerate(bodies):
            if b >= 0 and model.body_label[b] == 'S20/payload_box':
                flags[i] &= ~1
        model.shape_flags.assign(flags)
        newton.eval_fk(model,model.joint_q,model.joint_qd,model)
        container = next(t for t in config['static_transforms'] if t['child']=='container_link')
        return model,ids,container,config
    original = observed_cycle.make_model
    original_robot=native.make_robot_urdf
    if site_urdf is not None:
        path=ROOT/'experiments/s20_online_mixed8_20261024/pose_readback/measured_pose.py'
        spec=importlib.util.spec_from_file_location('current_site_model',path)
        site=importlib.util.module_from_spec(spec);spec.loader.exec_module(site)
        site_xml=site.site_empty_urdf(Path(site_urdf),ROOT/'reproductions/ludan_greedy_20261023')
        def current_robot(*args,**kwargs):
            xml=copy.deepcopy(site_xml)
            legacy=ET.fromstring(original_robot(*args,**kwargs))
            for element in legacy:
                if element.get('name') in ('payload_box','payload_grip_joint'):xml.append(copy.deepcopy(element))
            return ET.tostring(xml,encoding='unicode')
        native.make_robot_urdf=current_robot
    observed_cycle.make_model = factory
    try:
        runtime = observed_cycle.Runtime(output,False,catalog=catalog)
    finally:
        observed_cycle.make_model = original
        native.make_robot_urdf=original_robot
    runtime.site_urdf=str(Path(site_urdf).resolve()) if site_urdf else None
    runtime.sensor = SensorTiledCamera(runtime.model,config=RenderConfig(enable_textures=True,enable_shadows=False))
    runtime.sensor.utils.create_default_light(enable_shadows=False)
    runtime.suitcase_generation_metadata = dict(objects=generation,
        render_depth='authored suitcase triangle mesh with UV/PNG; same visible geometry for RGB and depth',
        collision='original invisible cuboid dimensions retained for CAD/FCL; explicit conservative proxy',
        asset_provenance='local classmate-repository vintage asset, not an NVIDIA official task asset',
        gt_mask_used=False,detector_receives_generation_metadata=False)
    return runtime


def apply_suitcase_visual(runtime,index):
    """Enable the prebuilt suitcase mesh; retain invisible collision proxy.

    Call make_runtime first: existing finalized cube-only runtimes cannot be
    silently relabeled or have their depth faked by an RGB overlay.
    """
    label = f'B{index+1}'
    names = runtime.model.shape_label
    if label+'_suitcase_visual' not in names:
        raise ValueError('Use suitcase_capture.make_runtime: mesh must be built before finalize')
    flags = runtime.model.shape_flags.numpy()
    flags[names.index(label+'_collision_box')] &= ~1
    flags[names.index(label+'_suitcase_visual')] |= 1
    runtime.model.shape_flags.assign(flags)
    runtime.model.bvh_build_shapes(runtime.state)
    return runtime.suitcase_generation_metadata['objects'][index]


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose-json',type=Path,required=True,help='measured OR main-provided raised q_rad/q_deg; used without IK')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--count',type=int,default=2)
    parser.add_argument('--seed',type=int,default=24)
    parser.add_argument('--centers-xy',type=json.loads,help='generation-only [[world_x,world_y],...]')
    parser.add_argument('--yaw-deg',type=float,nargs='+')
    parser.add_argument('--weights',type=Path,default=Path('/home/ubuntu/yolo11n-seg.pt'))
    parser.add_argument('--conf',type=float,default=.25)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    for key,folder in [('WARP_CACHE_PATH','warp_cache'),('YOLO_CONFIG_DIR','yolo_config'),('MPLCONFIGDIR','mpl_config'),('XDG_CACHE_HOME','cache')]:
        os.environ[key] = str(args.output/folder)
    import warp as wp
    wp.config.kernel_cache_dir = str(args.output/'warp_cache')
    from scipy.spatial.transform import Rotation as R
    from yolo_sim_perception import render_capture,infer
    from ultralytics import YOLO
    catalog = [[.55,.4,.25] if i%2==0 else [.65,.45,.30] for i in range(args.count)]
    runtime = make_runtime(args.output,catalog)
    pose = json.loads(args.pose_json.read_text())
    sample = pose.get('samples',[pose])[-1]
    q = np.array(pose.get('q_rad',sample.get('q_rad',[])),float)
    if not q.size:
        q = np.radians(pose.get('q_deg',sample.get('q_deg')))
    rng = np.random.default_rng(args.seed)
    centers = args.centers_xy or [[(i%2-.5)*.80,1.65+(i//2)*.85] for i in range(args.count)]
    yaws = args.yaw_deg or rng.uniform(-180,180,args.count).tolist()
    if len(centers)!=args.count or len(yaws)!=args.count:
        raise ValueError('Provide one center/yaw per ground box')
    for i,(body,dims) in enumerate(zip(runtime.boxes,catalog)):
        runtime.fixed[body] = np.r_[centers[i],[dims[2]/2],R.from_euler('z',yaws[i],degrees=True).as_quat()]
    runtime.initial_q = q.copy()
    runtime.frame(q,'MEASURED_OR_MAIN_RAISED_Q_CAPTURE_ONLY')
    for i in range(args.count):
        apply_suitcase_visual(runtime,i)
    generation = dict(runtime.suitcase_generation_metadata,pose_source=str(args.pose_json),q_rad=q.tolist(),
        centers_xy=centers,yaw_deg=yaws,seed=args.seed,camera_link_world=runtime.state.body_q.numpy()[runtime.camera].tolist())
    (args.output/'generation_metadata.json').write_text(json.dumps(generation,indent=2))
    rgb,cloud = render_capture(runtime,args.output)
    import cv2
    result,mask,fitted = infer(YOLO(str(args.weights)),rgb,cloud,device='cpu',confidence=args.conf)
    cv2.imwrite(str(args.output/'yolo_overlay.png'),infer.last_overlay)
    if mask is not None:
        cv2.imwrite(str(args.output/'yolo_selection.png'),mask.astype('uint8')*255)
    result.update(weights=str(args.weights),pose_source=str(args.pose_json),full_classmate_scene=True,
                  robot_motion_executed=False,pick_executed=False,generation_metadata_used_by_detector=False)
    (args.output/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(status=result['status'],luggage_detections=result['luggage_detections'],
                         raw_detections=result['raw_detections'],output=str(args.output))))


if __name__ == '__main__':
    main()
