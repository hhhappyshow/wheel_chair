"""Audit saved actor states; report containment without changing planner goals."""
import json
import argparse
from pathlib import Path
import continue_place as c

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run-folder',type=Path,default=Path(__file__).resolve().parent/'attempt1')
args=parser.parse_args();out=args.run_folder.resolve()
if not out.is_relative_to(Path(__file__).resolve().parent):parser.error('Run folder must remain inside straight_place')
report=json.loads((out/'result.json').read_text());source=Path(report['source'])
saved=c.np.load(out/'trajectory.npz');original=c.np.load(source/'trajectory.npz')
r=c.make_runtime(out,report['actor_source_catalog_m'])
assert r.model.body_label==list(saved['body_labels'])
held=r.model.body_label.index('B1');r.w.held=held
r.w.grip_offset=c.np.array(report['simulation_actual_grip_offset_m'])
r.w.grip_rot=c.R.from_quat(report['simulation_actual_grip_rotation_xyzw'])
r.collision.arrived=set(range(len(r.boxes)));r.collision.current_source=0
hits=[]
for index,(q,body_q) in enumerate(zip(saved['joint_q'],saved['body_q'])):
    r.state.body_q.assign(body_q)
    if str(saved['stage'][index])=='RELEASE_ACTUAL_FROZEN':r.w.held=None
    if not r.collision.validate(q,actual=True):
        hits.append(dict(index=index,stage=str(saved['stage'][index]),pairs=r.collision.last_hits))
tf=next(v for v in r.cfg['static_transforms'] if v['child']=='container_link')
cr=c.R.from_euler('xyz',tf['rotation_rpy']);h=r.hull
pose=saved['body_q'][-1,held];origin=c.np.array([h.door_x,h.inner_y0,h.floor_z])
center=cr.inv().apply(pose[:3]-tf['translation'])-origin
rotation=cr.inv()*c.R.from_quat(pose[3:]);extent=abs(rotation.as_matrix())@c.np.array(report['actor_source_catalog_m'][0])
lower=center-extent/2;upper=center+extent/2
inside=bool(lower[0]>=0 and lower[1]>=0 and lower[2]>=h.floor_at(upper[1]) and upper[0]<=h.length and upper[1]<=h.width and upper[2]<=h.height)
audit=dict(initial_body_state_exact=c.np.array_equal(saved['body_q'][0],original['body_q'][-1]),
    initial_joint_state_exact=c.np.array_equal(saved['joint_q'][0],original['joint_q'][-1]),
    same_body_labels=list(saved['body_labels'])==list(original['body_labels']),
    existing_unpicked_B2_pose_frozen=bool(c.np.all(saved['body_q'][:,15]==saved['body_q'][0,15])),
    reference_duration_s=float(saved['times'][-1]),frames=len(saved['stage']),
    independent_actual_saved_frame_CAD_rejections=hits,
    final_actual_actor_center_container_m=center.tolist(),final_actual_actor_aabb_dimensions_m=extent.tolist(),
    final_actual_actor_aabb_min_container_m=lower.tolist(),final_actual_actor_aabb_max_container_m=upper.tolist(),
    final_actual_actor_relative_rpy_deg=rotation.as_euler('xyz',degrees=True).tolist(),
    nominal_hull_side_outside_m=max(0.,float(-lower[1])),nominal_hull_floor_outside_m=max(0.,float(-lower[2])),
    final_actor_aabb_inside_nominal_hull=inside,
    frozen_release_completed=bool(report['completed'] and saved['stage'][-1]=='RELEASE_ACTUAL_FROZEN'),
    verdict=('Actual actor AABB is inside nominal hull and all sampled saved states passed original CAD. Frozen release only, no settling/dynamics proof.' if inside and not hits else 'Actor containment or sampled original CAD not fully passed; report measured clearances/hits. No truth-fed target correction or additional acceptance gate applied.'),
    trajectory_scope='Saved-state continuation only; original pickup retained separately, not reexecuted or claimed uninterrupted')
(out/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
print(json.dumps(audit,indent=2))
