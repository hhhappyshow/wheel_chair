"""Main-loop interface; import EnvelopeGraspPlanner and cache one instance.

planner = EnvelopeGraspPlanner(tool_urdf_or_json)
grasp = planner.plan(current_observed_box, container_center_world_m=[x,y,z])
pose = grasp['selected']['T_world_tcp'] if grasp['selected'] else None

Observed dimensions are [long, short, height] in metres. T_world_box is the
volume-center pose with X=long edge, Z=up. No robot calls or simulator imports.
The returned pose is nominal contact, not an approach or motion certificate.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from envelope_grasp import load_tool, solve_grasp


class EnvelopeGraspPlanner:
    def __init__(self, tool_urdf_or_json):
        self.parts, self.contact, self.fixture, self.tool = load_tool(tool_urdf_or_json)

    def plan(self, observed_box, container_center_world_m=None, margin_m=.005,
             angles=(0, 90, 180, 270), centered=True, preferred_fixture_direction_world=None):
        fitted = dict(observed_box)
        box = np.asarray(fitted['T_world_box'], float)
        dims = np.asarray(fitted['dimensions_m'], float)
        if 'top_center_world_m' not in fitted:
            fitted['top_center_world_m'] = (box[:3,3]+box[:3,2]*dims[2]/2).tolist()
        result = solve_grasp(fitted, self.parts, self.contact, self.fixture,
                             center_world=container_center_world_m, angles=angles, margin=margin_m,
                             centered=centered, preferred_fixture_direction_world=preferred_fixture_direction_world)
        for candidate in result['candidates']:
            tcp = np.array(candidate['T_world_tcp'])
            candidate['tcp_position_world_m'] = tcp[:3,3].tolist()
            candidate['tcp_quaternion_world_xyzw'] = Rotation.from_matrix(tcp[:3,:3]).as_quat().tolist()
            candidate['T_tcp_box_estimated'] = self.attachment_transform(tcp, box).tolist()
        result['observation_source'] = fitted.get('selection_source', fitted.get('source', 'unspecified'))
        result['complete_top_confirmed'] = bool(fitted.get('complete_top_confirmed', False))
        result['downstream_contract'] = ('Use the returned offset grasp pose. At achieved contact recompute inv(T_world_tcp_achieved) @ T_world_box_observed; '
            'carry this binding into placement, never substitute centered [0,0,height/2]. Completeness and IK/path decisions belong to the caller.')
        if centered:
            result['downstream_contract'] = ('TCP equals observed top center without translation; four edge-parallel orientations only. '
                'Use actual achieved TCP/observed box binding after contact. Direction preference is not collision validation.')
        return result

    @staticmethod
    def attachment_transform(T_world_tcp_achieved, T_world_box_observed):
        return np.linalg.inv(np.asarray(T_world_tcp_achieved)) @ np.asarray(T_world_box_observed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observed', type=Path, required=True)
    parser.add_argument('--tool', type=Path, required=True)
    parser.add_argument('--container-center', nargs=3, type=float)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--centered', dest='centered', action='store_true')
    modes.add_argument('--allow-offset', dest='centered', action='store_false')
    parser.set_defaults(centered=True)
    parser.add_argument('--preferred-fixture-direction-world', nargs=3, type=float)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    observed = json.loads(args.observed.read_text())
    result = EnvelopeGraspPlanner(args.tool).plan(observed, args.container_center,
        centered=args.centered, preferred_fixture_direction_world=args.preferred_fixture_direction_world)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(output=str(args.output), full_fit=bool(result['selected'] and result['selected']['full_envelope_fits']))))


if __name__ == '__main__':
    main()
