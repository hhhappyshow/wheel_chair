"""Current observed box -> Contact V2 geometric goals; no robot or simulator.

JSONL stdin keeps imports/config warm. Each request supplies its own committed
occupancy; proposing a goal never commits a box or authorizes execution.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from current_profile import load_profile

EXPERIMENT = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT.parents[1]
sys.path[:0] = [str(EXPERIMENT), str(ROOT/'experiments/s20_greedy_async_20261023')]
from compare import Box, p
from straight_place.contact_layout import ContactLayoutPrior


class CandidatePlanner:
    def __init__(self, scene):
        self.scene = Path(scene).resolve()
        self.config = yaml.safe_load(self.scene.read_text())
        self.hull = p.Hull.from_config(self.config)
        self.packing = load_profile()['packing']
        self.prior = ContactLayoutPrior(self.config, self.hull,
            use_endpoint_sphere=False, max_box_top_m=self.packing['max_box_top_m'],
            neighbor_gap_m=self.packing['worker_options']['neighbor_gap_m'])
        self.prior.lateral_clearance_m = self.packing['preview_lateral_clearance_m']
        self.prior.horizontal_row_first = self.packing['worker_options']['horizontal_row_first']
        self.prior.physical_padding_xy_m = self.packing['padding_xy_per_side_m']
        transform = next(t for t in self.config['static_transforms']
                         if t['child'] == 'container_link')
        self.rotation = Rotation.from_euler('xyz', transform['rotation_rpy'])
        self.translation = np.asarray(transform['translation'])
        self.origin = np.array([self.hull.door_x, self.hull.inner_y0, self.hull.floor_z])

    def plan(self, request):
        began = time.perf_counter()
        # No nominal20 fallback: every goal uses this observation's dimensions.
        observed_dimensions = tuple(float(v) for v in request['observed_dimensions_m'])
        padding = self.packing['padding_xy_per_side_m']
        dimensions = (observed_dimensions[0]+2*padding, observed_dimensions[1]+2*padding, observed_dimensions[2])
        occupied = [Box(b['id'], tuple(b['dims']), tuple(b['pos']))
                    for b in request['committed_occupancy']]
        candidates, statistics = self.prior.candidates(dimensions, occupied)
        goals = []
        for rank, box in enumerate(candidates, 1):
            yaw = 0. if np.allclose(box.dims, dimensions, rtol=0, atol=1e-9) else np.pi/2
            center = self.origin + np.asarray(box.pos) + np.asarray(box.dims)/2
            orientation = self.rotation * Rotation.from_euler('z', yaw)
            goals.append(dict(rank=rank, box_uld=asdict(box), yaw_uld_rad=yaw,
                box_center_scene_m=(self.translation+self.rotation.apply(center)).tolist(),
                box_rotation_scene_xyzw=orientation.as_quat().tolist(),
                top_center_scene_m=self.prior.top(box).tolist(),
                contact_score=self.prior.score(box, occupied),
                tool_pose_not_yet_solved=True))
        return dict(request_id=request['request_id'], scene_version=request['scene_version'],
            status='geometry_ready' if goals else 'unresolved', candidates=goals,
            observed_dimensions_m=observed_dimensions, planning_dimensions_m=dimensions,
            padding_xy_per_side_m=padding, committed_count=len(occupied),
            spatial_planning_s=time.perf_counter()-began, statistics=statistics,
            scene_config=str(self.scene), future_arrivals_used=0,
            motion_sent=False, occupancy_committed=False, trajectory_validated=False,
            note='Goals only: attachment-aware tool IK, full loaded/empty paths and actual-state revalidation remain required')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=Path(__file__).with_name('scene_tf.yaml'))
    parser.add_argument('--request', type=Path, help='One request file; otherwise JSONL stdin')
    args = parser.parse_args()
    planner = CandidatePlanner(args.scene)
    requests = [args.request.read_text()] if args.request else sys.stdin
    for line in requests:
        if not line.strip():
            continue
        try:
            result = planner.plan(json.loads(line))
        except Exception as exc:
            result = dict(status='unresolved', error=f'{type(exc).__name__}: {exc}',
                          motion_sent=False, occupancy_committed=False)
        print(json.dumps(result, default=lambda value: value.item() if isinstance(value, np.generic)
                         else value.tolist()), flush=True)


if __name__ == '__main__':
    main()
