"""Centered four-heading CPU audit, independent of historical offset outputs."""
import json
import os
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from envelope_grasp import plot_result
from grasp_interface import EnvelopeGraspPlanner


def main():
    out = Path(__file__).resolve().parent
    root = out.parents[2]
    os.environ['MPLCONFIGDIR'] = str(out/'centered/mpl_config')
    source = root/'experiments/s20_greedy_async_20261023'
    planner = EnvelopeGraspPlanner(source/'v2_s20_model_h100/s20_current.urdf')
    cfg = yaml.safe_load((root/'reproductions/ludan_greedy_20261023/deployment_ws/config/scene_tf.loading_site.yaml').read_text())
    frame = next(t for t in cfg['static_transforms'] if t['child'] == 'container_link')
    inward_world = Rotation.from_euler('xyz',frame['rotation_rpy']).apply([1,0,0])
    rows = []
    for label in ('B1','B2','B3','B4'):
        path = source/f'mixed8_horizontal_observed_attempt2/perception/{label}/observed_box.json'
        fitted = json.loads(path.read_text())
        result = planner.plan(fitted, centered=True, preferred_fixture_direction_world=inward_world)
        positive_x = [c for c in result['candidates'] if c['preferred_fixture_direction_score'] > 0 and c['contact_plate_fits']]
        constrained = min(positive_x,key=lambda c:(not c['full_envelope_fits'],c['max_overhang_m'],-c['preferred_fixture_direction_score'])) if positive_x else None
        result['container_positive_x_constrained'] = constrained
        result['direction_constraint'] = 'fixture-side dot container +X > 0; directional half-space only, no collision guarantee'
        folder = out/'centered'/label
        folder.mkdir(parents=True,exist_ok=True)
        (folder/'result.json').write_text(json.dumps(result,indent=2))
        if label in ('B1','B2'):
            plot_result(fitted,result,folder/'coverage.png')
        errors = [float(np.max(abs(np.array(c['T_world_tcp'])[:3,3]-fitted['top_center_world_m']))) for c in result['candidates']]
        rows.append(dict(box=label,dimensions_m=fitted['dimensions_m'],
            tcp_top_center_max_error_m=max(errors),
            headings=[dict(angle_deg=c['angle_deg'],offset_xy_m=c['tcp_offset_box_xy_m'],
                full_fit=c['full_envelope_fits'], overhang_minus_x_minus_y_plus_x_plus_y_m=c['overhang_minus_x_minus_y_plus_x_plus_y_m'],
                container_positive_x_score=c['preferred_fixture_direction_score']) for c in result['candidates']],
            unconstrained_angle=result['selected']['angle_deg'] if result['selected'] else None,
            positive_container_x_angle=constrained['angle_deg'] if constrained else None))
    summary = dict(mode='centered_default', geometric_source_not_yolo=True,
        container_inward_axis_local=[1,0,0],container_inward_axis_world=inward_world.tolist(),
        full_existing_TF_retained=True,direction_is_not_collision_proof=True,
        gpu_used=False,results=rows,
        historical_offset_results='replay_B1..B4/, oversize_negative.json/png, audit.json, interface_example.json retained untouched')
    (out/'centered_summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
