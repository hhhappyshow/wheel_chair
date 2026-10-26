"""Observable planner decisions, not private reasoning or a new packing policy.

An opt-in sink records candidates emitted by the actual sparse generator.
Rejection explanations are derived after selection; planner order is unchanged.
This diagnostic performs no IK, robot movement, learning, or physics execution.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np
import yaml

HERE=Path(__file__).resolve().parent.parent
ROOT=HERE.parent.parent
sys.path[:0]=[str(HERE),str(ROOT/'experiments/s20_greedy_async_20261023')]
from compare import Box,SCENE,p
from straight_place.sparse_prior import SparseCompactPrior
from push_compare.support_geometry_v2 import support_details


def identity(box):
    return (tuple(box.pos),tuple(box.dims))


def snapshot(prior,data):
    placed=data['placed'];h=prior.hull
    geometric={identity(b) for b in data['geometric']}
    eligible={identity(b) for b in data['eligible']}
    chosen={identity(b):i+1 for i,b in enumerate(data['selected'])}
    boxes=[Box('current',dims,pos) for pos,dims in sorted(data['tried'])]
    records=[]
    for index,b in enumerate(boxes):
        token=identity(b);reasons=[];support=support_details(b,placed,h,prior.min_support_coverage)
        if not h.fits(b):reasons.append('outside_hull')
        if b.pos[1]<prior.lateral_clearance_m-1e-7:reasons.append('wall_clearance')
        if prior.lateral_bounds and (b.pos[1]<prior.lateral_bounds[0]-1e-7 or b.pos[1]+b.dims[1]>prior.lateral_bounds[1]+1e-7):reasons.append('outside_lateral_bounds')
        if prior.max_box_top_m is not None and b.pos[2]+b.dims[2]>prior.max_box_top_m+p.EPS:reasons.append('height_cap')
        overlaps=[v.id for v in placed if p.collides(b,v)]
        if overlaps:reasons.append('box_overlap')
        if not support['supported']:reasons.append('insufficient_support')
        radius=min(np.linalg.norm(prior.top(b)+prior.rotation.apply(offset)-prior.center) for offset in prior.grasp_offsets(b.dims))
        if prior.use_endpoint_sphere and radius>prior.radius+1e-7:reasons.append('outside_coarse_sphere')
        if token in geometric and token not in eligible:
            if not prior._exposed(b,placed):reasons.append('hidden_insertion_lane')
            elif placed and not prior._connected(b,placed) and not prior._floor_frontier(b,placed):reasons.append('detached_from_frontier')
            else:reasons.append('deferred_floor_fallback')
        if token in eligible and token not in chosen:reasons.append('not_in_shortlist')
        rank=chosen.get(token)
        status='chosen' if rank==1 else 'shortlisted' if rank else 'eligible' if token in eligible else 'rejected'
        neighbor,wall=prior.contacts(b,placed)
        records.append(dict(id=f'C{index+1:03}',box=asdict(b),center=(np.asarray(b.pos)+np.asarray(b.dims)/2).tolist(),
            status=status,shortlist_rank=rank,reasons=reasons,overlap_ids=overlaps,support=support,
            sort_key=prior._rank(b,placed,'depth',prior.ordering),neighbor_contact_m2=neighbor,wall_contact_m2=wall,
            geometric_valid=token in geometric,frontier_eligible=token in eligible,
            top_center_base_radius_m=float(radius)))
    counts={status:sum(c['status']==status for c in records) for status in ('chosen','shortlisted','eligible','rejected')}
    return dict(current_dimensions_m=list(data['dims']),occupied=[asdict(b) for b in placed],
        candidates=records,stats=data['stats'],counts=counts,
        shortlist=[next(c['id'] for c in records if identity(Box(**c['box']))==identity(b)) for b in data['selected']],
        trace_scope='Unique materialized proposals only; early y/height/empty reach-interval pruning is not reconstructed as 3D points')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seed',type=int,default=73)
    parser.add_argument('--count',type=int,default=8)
    parser.add_argument('--method',choices=('frontier','contact'),default='frontier')
    parser.add_argument('--size-mix',choices=('20','20/24'),default='20')
    parser.add_argument('--jitter',type=float,default=.08)
    a=parser.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load(SCENE.read_text());h=p.Hull.from_config(cfg)
    result=dict(schema='s20_candidate_decisions/v1',seed=a.seed,arrivals=a.count,
        scope=__doc__,sphere_enabled=False,max_height_m=1.,axes=dict(x='entrance 0 -> rear',y='right 0 -> left',z='floor -> up'),
        active_method=a.method,methods=[],hardware_motion=False,future_arrivals_used=0,
        size_mix=a.size_mix,axis_jitter_fraction=a.jitter,
        output_is_geometric_diagnostic=True,performance_benchmark=False,
        explanation='Recorded lexicographic ranking plus greedy first choice, not a learned policy. Contact V2 ranks height/depth-band/right-band before contact and envelope growth; no arm execution.')
    from straight_place.contact_layout import ContactLayoutPrior
    for name,cls in ((a.method,ContactLayoutPrior if a.method=='contact' else SparseCompactPrior),):
        prior=cls(cfg,h,use_endpoint_sphere=False,max_box_top_m=1.)
        prior.lateral_clearance_m=0.;rng=np.random.default_rng(a.seed);placed=[];steps=[]
        def sink(data):steps.append(snapshot(prior,data))
        prior.trace_sink=sink
        for index in range(a.count):
            size_class=int(rng.choice([20,24])) if a.size_mix=='20/24' else 20
            dims=np.asarray([.55,.4,.25] if size_class==20 else [.65,.45,.28])*rng.uniform(1-a.jitter,1+a.jitter,3)
            candidates,stats=prior.candidates(dims,placed)
            step=steps[-1];step['arrival']=index+1;step['accepted']=bool(candidates)
            step['size_class_inches']=size_class
            if candidates:
                b=candidates[0];b=Box(f'B{index+1}',tuple(b.dims),tuple(b.pos));placed.append(b)
                step['selected_box']=asdict(b)
        result['methods'].append(dict(name=name,accepted=len(placed),steps=steps))
    (a.output/'trace.json').write_text(json.dumps(result,indent=2,default=lambda v:v.item() if isinstance(v,np.generic) else v.tolist()))
    print(a.output/'trace.json')


if __name__=='__main__':main()
