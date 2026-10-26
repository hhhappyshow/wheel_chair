"""Prepare an empty-arm ServoJ return before taking a fresh observation."""
import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(HERE.parent), str(HERE.parent/'perception'),
               str(ROOT/'experiments/s20_greedy_async_20261023')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot', type=Path, required=True)
    p.add_argument('--scene-state', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--vacuum-feedback',choices=('digital','operator'),default='digital')
    a = p.parse_args()
    import numpy as np
    snapshot = json.loads(a.snapshot.read_text())
    if a.vacuum_feedback=='digital' and snapshot['vacuum']['DI0']:
        raise RuntimeError('Holding feedback is active; empty-arm restart is not applicable')
    scene = json.loads(a.scene_state.read_text()) if a.scene_state else {}
    points = json.loads((HERE/'pick_waypoints.json').read_text())
    a.output.mkdir(parents=True, exist_ok=False)
    q0 = np.array(snapshot['q_rad'])
    q1 = np.array(points['observation']['q_rad'])
    error_deg=float(np.max(np.abs(np.degrees(q0-q1))))
    # Pose equivalence avoids reinitializing the model for encoder-level drift;
    # it does not reject motion or relax collision checks on an actual return.
    if error_deg<=.05:
        recipe=dict(role='empty_observation_return',operations=[],already_at_observation=True,
            max_joint_error_deg=error_deg,pose_equivalence_deg=.05)
        (a.output/'cycle.json').write_text(json.dumps(recipe,indent=2))
        print(json.dumps(recipe))
        return
    from suitcase_capture import make_runtime
    from observed_collision import Collision
    from servoj_reference import prepare_reference
    r = make_runtime(a.output/'runtime', scene.get('catalog') or [[.55,.40,.25]],
                     site_urdf=HERE/'site_robot.urdf')
    r.collision = Collision(r.w)
    r.fixed = {r.boxes[i]: np.array(scene['placed_world_poses'][f'B{i+1}'])
               for i in range(len(scene.get('catalog', [])))}
    r.collision.arrived = set(range(len(scene.get('catalog', []))))
    r.frame(q0, 'MEASURED_START')
    zeros = np.zeros((2,6))
    reference = prepare_reference([0,1], np.array([q0,q1]),
        velocities_rad_s=zeros, accelerations_rad_s2=zeros, phase='normal')
    for q in np.radians(reference['positions_deg']):
        if not r.validate(q):
            raise RuntimeError(f'Observation return intersects modeled CAD: {r.collision.last_hits}')
    (a.output/'return.json').write_text(json.dumps(dict(
        period_s=reference['servo_time_s'], positions_deg=reference['positions_deg'],
        velocities_deg_s=reference['velocities_deg_s'],velocity_source='analytic_source_curve')))
    recipe = dict(role='empty_observation_return', operations=[dict(type='servoj',
        name='return_to_observation', reference='return.json', duration_s=reference['duration_s'])],
        environment_scope='Known CAD and committed boxes; no fresh floor-object observation yet')
    (a.output/'cycle.json').write_text(json.dumps(recipe, indent=2))
    print(json.dumps(dict(output=str(a.output), reference_s=reference['duration_s'], motion_sent=False)))


if __name__ == '__main__':
    main()
