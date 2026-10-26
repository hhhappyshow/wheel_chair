"""Online current-box layout only, followed by native Newton static rendering.

No perception, arm, IK, motion, dynamics, RL, or future-box lookahead. Random
dimensions are sampled only when a box arrives; accepted boxes are immutable.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

HERE=Path(__file__).resolve().parent.parent
ROOT=HERE.parent.parent
sys.path[:0]=[str(HERE),str(ROOT/'experiments/s20_greedy_async_20261023')]
from compare import Box,SCENE,UPSTREAM,p
from straight_place.sparse_prior import SparseCompactPrior
from push_compare.support_geometry_v2 import support_details


def online(seed,count,jitter,method,reach_sphere,size_mix='20'):
    cfg=yaml.safe_load(SCENE.read_text());h=p.Hull.from_config(cfg)
    if method=='contact':
        from straight_place.contact_layout import ContactLayoutPrior
        prior=ContactLayoutPrior(cfg,h,use_endpoint_sphere=reach_sphere,max_box_top_m=1.)
    else:
        prior=SparseCompactPrior(cfg,h,use_endpoint_sphere=reach_sphere,max_box_top_m=1.)
    # Ideal geometric touching: do not insert a cosmetic inter-box gap.
    prior.lateral_clearance_m=0.
    rng=np.random.default_rng(seed);placed=[];arrivals=[]
    for index in range(count):
        size_class=int(rng.choice([20,24])) if size_mix=='20/24' else 20
        nominal=[.55,.4,.25] if size_class==20 else [.65,.45,.28]
        dims=tuple(np.array(nominal)*rng.uniform(1-jitter,1+jitter,3))
        began=time.perf_counter()
        candidates,stats=prior.candidates(dims,placed)
        row=dict(index=index+1,size_class_inches=size_class,nominal_dimensions_m=nominal,
                 dimensions_m=list(dims),candidate_stats=stats,accepted=False,
                 occupied_count_at_query=len(placed),future_arrivals_used=0)
        if candidates:
            chosen=candidates[0];chosen=Box(f'B{index+1}',chosen.dims,chosen.pos)
            row.update(accepted=True,box=asdict(chosen),support=support_details(chosen,placed,h),
                       face_contact_m2=prior.contacts(chosen,placed)[0],wall_contact_m2=prior.contacts(chosen,placed)[1])
            placed.append(chosen)
        row['planning_s']=time.perf_counter()-began
        arrivals.append(row)
    times=np.array([a['planning_s'] for a in arrivals])
    return dict(seed=seed,method=method,requested=count,accepted=len(placed),arrivals=arrivals,
        max_candidates=10,initial_size_prior_m=[.55,.4,.25],axis_jitter_fraction=jitter,
        size_mix=size_mix,nominal_sizes_m={'20':[.55,.4,.25],'24':[.65,.45,.28]},
        size_model='Proxy dimensions, not measured luggage; independent per-axis uniform size variation, not pose or perception noise',
        hidden_future_used=False,preloaded_trajectory=False,boxes_rearranged=False,
        endpoint_sphere_enabled=reach_sphere,max_box_top_m=1.,
        mean_planning_ms=float(times.mean()*1000),max_planning_ms=float(times.max()*1000),
        scope=__doc__,scene_config=str(SCENE),render_is_static_geometry_not_execution=True)


def render(result,out):
    import newton
    import newton.viewer
    import warp as wp
    import trimesh
    from PIL import Image
    from online_packing_newton import make_model
    cfg=yaml.safe_load(SCENE.read_text());h=p.Hull.from_config(cfg)
    accepted=[v for v in result['arrivals'] if v['accepted']]
    sample=dict(container=h.as_dict(),steps=[dict(incoming=dict(id=a['box']['id'],dims=a['box']['dims'])) for a in accepted])
    wp.set_device('cuda:0')
    model,ids,_,_=make_model(sample,UPSTREAM/'policy/vendor/roboticarm',SCENE,show_robot=False)
    state=model.state();poses=model.body_q.numpy().copy()
    tf=next(t for t in cfg['static_transforms'] if t['child']=='container_link')
    rotation=R.from_euler('xyz',tf['rotation_rpy']);translation=np.asarray(tf['translation'])
    origin=np.array([h.door_x,h.inner_y0,h.floor_z])
    for a in accepted:
        b=a['box'];center=np.asarray(b['pos'])+np.asarray(b['dims'])/2+origin
        # Yaw is already encoded by swapped axis-aligned dimensions.
        poses[ids[b['id']]]=np.r_[translation+rotation.apply(center),rotation.as_quat()]
    state.body_q.assign(poses)
    colors=model.shape_color.numpy();bodies=model.shape_body.numpy()
    palette=[(.12,.62,.69),(.93,.58,.19),(.58,.4,.72),(.33,.68,.43),(.83,.33,.4),(.28,.5,.79),(.77,.68,.24),(.44,.69,.73)]
    for shape,body in enumerate(bodies):
        label=model.body_label[body] if body>=0 else ''
        if label in ids:colors[shape]=palette[(int(label[1:])-1)%len(palette)]
    model.shape_color.assign(colors)
    flags=model.shape_flags.numpy()
    for shape,body in enumerate(bodies):
        label=model.shape_label[shape].lower()
        if (body>=0 and model.body_label[body].startswith('S20/')) or 'pedestal' in label or 'pickup' in label:
            flags[shape] &= ~1
    model.shape_flags.assign(flags)
    viewer=newton.viewer.ViewerGL(width=1100,height=850,headless=True)
    viewer.set_model(model);viewer.show_ui=False
    shell=model.shape_label.index('container_collision_full');visual=model.shape_label.index('container_visual')
    source=model.shape_source[shell];xf=model.shape_transform.numpy()[shell]
    mesh=trimesh.Trimesh(np.asarray(source._vertices)*model.shape_scale.numpy()[shell],np.asarray(source._indices).reshape(-1,3),process=True)
    edges=mesh.face_adjacency_edges[mesh.face_adjacency_angles>.2]
    vertices=R.from_quat(xf[3:]).apply(mesh.vertices)+xf[:3]
    starts=wp.array(vertices[edges[:,0]].astype(np.float32),dtype=wp.vec3)
    ends=wp.array(vertices[edges[:,1]].astype(np.float32),dtype=wp.vec3)
    overlays=[]
    edge_pairs=np.array([(i,j) for i in range(8) for j in range(i+1,8) if (i^j) in (1,2,4)])
    unit=np.array([[i&1,(i>>1)&1,(i>>2)&1] for i in range(8)])
    for index,overlay in enumerate(result.get('candidate_overlays',[])):
        b=overlay['box']
        corners=translation+rotation.apply(np.asarray(b['pos'])+unit*np.asarray(b['dims'])+origin)
        overlays.append((f'candidate_{index}',wp.array(corners[edge_pairs[:,0]].astype(np.float32),dtype=wp.vec3),
                         wp.array(corners[edge_pairs[:,1]].astype(np.float32),dtype=wp.vec3),tuple(overlay['color'])))
    target=translation+rotation.apply([0.,-.1,1.05])
    cameras=[target+np.array([2.6,2.9,2.4]),target+np.array([2.6,-2.9,2.4])]
    paths=[]
    for index,camera in enumerate(cameras):
        delta=target-camera;pitch=np.degrees(np.arctan2(delta[2],np.linalg.norm(delta[:2])));yaw=np.degrees(np.arctan2(delta[1],delta[0]))
        viewer.set_camera(pos=wp.vec3(*camera),pitch=pitch,yaw=yaw)
        for _ in range(3):
            viewer.begin_frame(0.);viewer.log_state(state)
            viewer.objects[viewer._shape_to_batch[visual].name].hidden=True
            viewer.log_lines('cad_shell',starts,ends,(.45,.65,.75))
            for name,a,b,color in overlays:viewer.log_lines(name,a,b,color)
            viewer.end_frame()
        path=out/f"{result['method']}_seed{result['seed']}_view{index+1}.png"
        Image.fromarray(viewer.get_frame().numpy()).save(path);paths.append(str(path.resolve()))
    viewer.close();return paths


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--count',type=int,default=8)
    parser.add_argument('--seeds',nargs='+',type=int,default=[73,74,75])
    parser.add_argument('--jitter',type=float,default=.08)
    parser.add_argument('--size-mix',choices=('20','20/24'),default='20')
    parser.add_argument('--methods',nargs='+',choices=('frontier','contact'),default=['frontier'],
                        help='Known frontier baseline by default; contact remains an explicit archived experiment')
    parser.add_argument('--reach-sphere',action='store_true')
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--render-result',type=int,help='Render one saved result in a fresh GL process; no new planning')
    a=parser.parse_args()
    if a.render_result is not None:
        results=json.loads((a.output/'results.json').read_text())
        print(render(results[a.render_result],a.output));return
    a.output.mkdir(parents=True,exist_ok=False)
    results=[]
    for seed in a.seeds:
        for method in a.methods:
            result=online(seed,a.count,a.jitter,method,a.reach_sphere,a.size_mix)
            results.append(result)
            (a.output/'results.json').write_text(json.dumps(results,indent=2,default=lambda v:v.item() if isinstance(v,np.generic) else v.tolist()))
            if a.render:
                # A fresh process per static scene avoids stale GL contexts
                # when ViewerGL.close terminates the previous window backend.
                subprocess.run([sys.executable,str(Path(__file__).resolve()),'--output',str(a.output.resolve()),
                    '--render-result',str(len(results)-1)],check=True)
                result['images']=[str((a.output/f'{method}_seed{seed}_view{k}.png').resolve()) for k in (1,2)]
                (a.output/'results.json').write_text(json.dumps(results,indent=2,default=lambda v:v.item() if isinstance(v,np.generic) else v.tolist()))
            print(json.dumps({key:result[key] for key in ('seed','method','requested','accepted','mean_planning_ms','max_planning_ms')}),flush=True)


if __name__=='__main__':main()
