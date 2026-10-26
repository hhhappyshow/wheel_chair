"""Re-fit saved geometric observations and audit actual fixed CAD, on CPU."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import pickletools
import sys
import zipfile
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation
from envelope_grasp import load_tool, fit_box, solve_grasp, plot_result, transform


def weight_metadata(path):
    # Inspect the pickle instruction stream without unpickling model objects.
    with zipfile.ZipFile(path) as archive:
        ops = list(pickletools.genops(archive.read(next(n for n in archive.namelist() if n.endswith('/data.pkl')))))
    strings = [(i, a) for i, (_, a, _) in enumerate(ops) if isinstance(a, str)]
    names_index = next(i for i, a in strings if a == 'names')
    end = next(i for i in range(names_index+1, len(ops)) if ops[i][0].name == 'SETITEMS')
    names = {}
    last_integer = None
    for op, arg, _ in ops[names_index:end]:
        if op.name in ('BININT', 'BININT1', 'BININT2', 'INT'):
            last_integer = arg
        elif op.name == 'BINUNICODE' and arg != 'names' and last_integer is not None:
            names[last_integer] = arg
            last_integer = None
    return dict(path=str(path), inspection='zip/pickle opcode metadata only; no torch import or forward',
                class_count=len(names), luggage_classes={k: v for k, v in names.items() if v in ('suitcase', 'luggage')},
                task_segment=any(a == 'segment' for _, a in strings),
                coco_dataset=any('datasets/coco.yaml' in a for _, a in strings),
                size_20_24_classifier=False,
                limitation='COCO suitcase class exists; accuracy on plain Newton cuboids is untested')


def main():
    out = Path(__file__).resolve().parent
    root = out.parents[2]
    os.environ['MPLCONFIGDIR'] = str(out/'mpl_config')
    source = root/'experiments/s20_greedy_async_20261023'
    urdf = source/'v2_s20_model_h100/s20_current.urdf'
    parts, contact, fixture, tool = load_tool(urdf)
    scene = root/'reproductions/ludan_greedy_20261023/deployment_ws/config/scene_tf.loading_site.yaml'
    import yaml
    cfg = yaml.safe_load(scene.read_text())
    container = next(t for t in cfg['static_transforms'] if t['child'] == 'container_link')
    # A point on the container centerline (not a changed calibration).
    center_world = np.asarray(container['translation'])+Rotation.from_euler('xyz', container['rotation_rpy']).apply(
        [0, 0, (cfg['container']['inner']['floor_z']+cfg['container']['inner']['ceiling_z'])/2])
    cases, rotations = [], []
    originals = []
    for boxid in ('B1', 'B2', 'B3', 'B4'):
        folder = source/'mixed8_horizontal_observed_attempt2/perception'/boxid
        observation = folder/'camera_observation.npz'
        if not observation.exists():
            continue
        data = np.load(observation)
        mask = data['selected'].astype(bool)
        cloud = data['points_world'][mask][::2]
        fitted = fit_box(cloud, data['ground_plane'])
        old = json.loads((folder/'observed_box.json').read_text())
        solution = solve_grasp(fitted, parts, contact, fixture, center_world, angles=(0,180), centered=False)
        case_out = out/f'replay_{boxid}'
        case_out.mkdir(exist_ok=True)
        result = dict(fitted=fitted, solution=solution, source_npz=str(observation),
            selection_source='saved depth geometry connected-component, NOT YOLO or GT mask',
            image_border_clipped=bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any()),
            complete_top_confirmed=False, contact_executed=False,
            dimension_difference_from_saved_fit_m=(np.array(fitted['dimensions_m'])-old['dimensions_m']).tolist())
        if boxid == 'B1':
            result['tool_coordinates'] = tool
            result['component_bounds_tcp_m'] = [dict(name=p['name'], bounds=[
                p['vertices'].min(axis=0).tolist(), p['vertices'].max(axis=0).tolist()]) for p in parts]
        (case_out/'result.json').write_text(json.dumps(result, indent=2))
        plot_result(fitted, solution, case_out/'coverage.png')
        chosen = solution['selected']
        cases.append(dict(box=boxid, dimensions_m=fitted['dimensions_m'], yaw_deg=fitted['yaw_world_deg'],
            full_fit=bool(chosen and chosen['full_envelope_fits']),
            selected_angle_deg=chosen['angle_deg'] if chosen else None,
            tcp_offset_box_xy_m=chosen['tcp_offset_box_xy_m'] if chosen else None,
            envelope_span_box_xy_m=chosen['envelope_span_box_xy_m'] if chosen else None,
            all_aligned_poses=[c['long_axis_aligned'] for c in solution['candidates']],
            dimensions_delta_max_m=float(np.max(abs(np.array(fitted['dimensions_m'])-old['dimensions_m'])))))
        originals.append((cloud, data['ground_plane'], fitted))
    # CPU rigid rotations of recorded depth: algorithm covariance only, not
    # fresh random-yaw camera observations or YOLO generalization evidence.
    rng = np.random.default_rng(24)
    for yaw in rng.uniform(-180, 180, 6):
        cloud, ground, old = originals[0]
        tf = np.eye(4)
        tf[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        moved = fit_box(transform(cloud, tf), np.linalg.inv(tf).T @ ground)
        solution = solve_grasp(moved, parts, contact, fixture, transform([center_world], tf)[0], angles=(0,180), centered=False)
        rotations.append(dict(added_yaw_deg=float(yaw),
            max_dimension_drift_m=float(np.max(abs(np.array(moved['dimensions_m'])-old['dimensions_m']))),
            full_fit=bool(solution['selected'] and solution['selected']['full_envelope_fits'])))
    small = copy.deepcopy(originals[0][2])
    small['dimensions_m'] = [.30, .24, .20]
    small_tf = np.array(small['T_world_box'])
    small_tf[:3, 3] = np.array(small['top_center_world_m'])-small_tf[:3, 2]*.10
    small['T_world_box'] = small_tf.tolist()
    small['source'] = 'synthetic undersized-face negative case with actual tool CAD; not observed luggage'
    negative = solve_grasp(small, parts, contact, fixture, center_world, (0, 90, 180, 270), centered=False)
    (out/'oversize_negative.json').write_text(json.dumps(dict(fitted=small, solution=negative), indent=2))
    plot_result(small, negative, out/'oversize_negative.png')
    # Match flattened snapshot against CURRENT source xacro properties, without
    # importing roboticarm_newton (which imports GPU libraries).
    current = root/'reproductions/ludan_greedy_20261023/policy/vendor/roboticarm/elfin_humble_ws/src/luggage_description/config'
    xml = ET.parse(urdf).getroot()
    matches = {}
    for filename, joint_name, prefix in (
        ('suction_flange_origin.xacro', 'suction_panel_joint', 'suction_flange'),
        ('eef_mount_adapter_origin.xacro', 'eef_mount_adapter_joint', 'adapter_mount'),
        ('camera_mount_origin.xacro', 'camera_mount_joint', 'cam_mount'),
        ('mid360_origin.xacro', 'mid360_mount_joint', 'mid360_mount')):
        props = {e.get('name'): e.get('value') for e in ET.parse(current/filename).getroot() if e.get('name')}
        j = xml.find(f"joint[@name='{joint_name}']")
        matches[joint_name] = dict(source=str(current/filename),
            xyz_matches=bool(np.allclose(np.fromstring(props[prefix+'_xyz'], sep=' '), np.fromstring(j.find('origin').get('xyz'), sep=' '))),
            rpy_matches=bool(np.allclose(np.fromstring(props[prefix+'_rpy'], sep=' '), np.fromstring(j.find('origin').get('rpy'), sep=' '))))
    weight = weight_metadata(Path('/home/ubuntu/yolo11n-seg.pt'))
    commands = dict(status='NOT_EXECUTED_GPU_WAITING_FOR_MAIN_PROCESS_NOTIFICATION',
        dependencies='one interpreter with Newton/Warp, Ultralytics/Torch, cv2, scipy, matplotlib, PyYAML; checked Newton env lacks Torch/Ultralytics/cv2/matplotlib',
        gpu_commands=[dict(size=size, argv=[str(root/'tools/newton/.venv/bin/python'),
          str(out/'yolo_sim_perception.py'), '--weights', '/home/ubuntu/yolo11n-seg.pt',
          '--render', '--device', 'cuda:0', '--size', str(size), '--seed', str(seed),
          '--tool', str(urdf), '--container-center', *map(str, center_world),
          '--output', str(out/f'gpu_yolo_{size}_s{seed}')]) for size,seed in ((20,24),(24,25))],
        no_motion_to_robot=True, no_pick_or_attach_even_in_simulation=True,
        cpu_reproduce=['python3', str(out/'cpu_audit.py')])
    (out/'commands.json').write_text(json.dumps(commands, indent=2))
    audit = dict(scope='CPU-only saved point clouds/CAD and checkpoint metadata', gpu_started=False,
        hardware_connected=False, source_repository_commit='1f0600802ed6c4b7beb4b4187aba4c2f299449b6',
        sources=dict(observed_cycle=str(source/'observed_cycle.py'), compare=str(source/'compare.py'),
                     newton_compare=str(source/'newton_compare.py'),
                     manual_workflow=str(root/'tools/three_box_original/manual_pick_place.py')),
        existing_perception=dict(observed_cycle='geometric elevated depth connected component, not YOLO',
          mixed_perception='same geometric model; complete_view based only on no image border clipping',
          manual_workflow='requires stable complete view, clear patch, valid top and height; imports hardware code, not executed',
          classmate_config='yolov8s-world.pt configured, not found locally in weight search; config is not a working YOLO forward'),
        tool=dict(urdf=str(urdf), detailed_coordinates=str(out/'replay_B1/result.json'), snapshot_current_xacro_match=matches,
          p04_span_tcp_xyz_m=np.ptp(contact, axis=0).tolist(),
          p04_z_bounds_tcp_m=[float(contact[:,2].min()), float(contact[:,2].max())],
          contact_plane_note='URDF nominal TCP Z=0; modeled p04 ends about 3mm behind it; no rubber seal validation or TCP change',
          fixture_side_tcp_m=fixture.tolist(),
          orientation='p04 square; align longest whole-assembly span along a p04 edge with box long edge; 180 symmetry prefers sensors toward container centerline'),
        centerline_preference_point_world_m=center_world.tolist(), saved_observation_results=cases,
        cpu_rotated_cloud_results=rotations,
        negative=dict(synthetic_top_size_m=[.30,.24], full_fit=negative['selected']['full_envelope_fits'],
          max_overhang_m=negative['selected']['max_overhang_m'], status=negative['selected']['containment_status'],
          ninety_degree_candidates_excluded_from_selection=True),
        weights=weight, cpu_runtime={n: bool(importlib.util.find_spec(n)) for n in ('ultralytics','torch','numpy','scipy','cv2')},
        camera_assets=dict(native_camera='Newton SensorTiledCamera 640x360, range-depth ray reconstruction; camera_link +X -> GL -Z; origin offset +0.025m X',
          simulated_arrivals='current Newton make_model uses colored cuboids, no luggage texture',
          textured_luggage_available=str(root/'reproductions/ludan_greedy_20261023/src/luggage_gazebo/models/suitcase_vintage/meshes/suitcase.obj'),
          textured_luggage_loaded_by_this_entry=False),
        limitations=['No YOLO forward ran: missing Ultralytics/Torch in checked CPU and Newton environments.',
          'COCO suitcase exists but no 20/24 size classifier or locally found suitcase-domain fine tune.',
          'Saved depth uses geometric mask; cannot count as YOLO result.',
          'No complete-top certification: no border clipping alone cannot rule out occlusion.',
          'CPU rotated clouds do not reproduce camera occlusion changes.',
          'No IK, swept collisions, vacuum seal, natural dynamics or hardware validation.',
          'Explicit tool JSON can override CAD/TCP; package:// URDF mesh paths need flattening first.'])
    (out/'audit.json').write_text(json.dumps(audit, indent=2))
    print(json.dumps(dict(saved_cases=cases, negative=audit['negative'], weights=weight)))


if __name__ == '__main__':
    main()
