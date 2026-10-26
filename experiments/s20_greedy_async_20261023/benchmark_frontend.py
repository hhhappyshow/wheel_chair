"""Measure AABB fitting and greedy-region search on actual exported depth points."""
import argparse
import json
import os
from pathlib import Path
import platform
import time
import numpy as np
import yaml
from compare import UPSTREAM,SCENE,p,top_surface_model,pool
from region_search import region_candidates


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--observation",type=Path,required=True);parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args();data=np.load(args.observation)
    points=data['points_world'];mask=data['selected'];sample=np.zeros_like(mask);sample[::2,::2]=True
    points=points[mask&sample];ground=data['ground_plane']
    timings=[];model=None
    for _ in range(12):
        start=time.perf_counter();model=top_surface_model(points,ground);timings.append(time.perf_counter()-start)
    hull=p.Hull.from_config(yaml.safe_load(SCENE.read_text()));domain=json.loads((UPSTREAM/'policy/demo_data/site_four_colors_right160_observe120/domain.json').read_text())['poses']
    scenarios=[[],[p.Box('prior',(.55,.4,.25),(.325,.33,0.))],[p.Box('prior',(.55,.4,.25),(.325,.33,0.)),p.Box('prior2',(.55,.4,.25),(.325,.33,.25))]]
    results=[]
    for placed in scenarios:
        raw=[];region=[];stats=None
        for _ in range(20):
            start=time.perf_counter();rows=pool(model['dimensions_m'],placed,hull);raw.append(time.perf_counter()-start)
            rows,stats=region_candidates(tuple(model['dimensions_m']),placed,hull,domain);region.append(stats['total_s'])
        results.append(dict(prefix=len(placed),raw_pool_median_ms=float(np.median(raw)*1000),region_median_ms=float(np.median(region)*1000),region_p95_ms=float(np.percentile(region,95)*1000),stats=stats))
    result=dict(scope=__doc__,machine=platform.node(),arch=platform.machine(),points=len(points),observed_model=model,
                model_median_ms=float(np.median(timings)*1000),model_p95_ms=float(np.percentile(timings,95)*1000),regions=results,
                model_first_ms=timings[0]*1000,model_warm_median_ms=float(np.median(timings[1:])*1000),model_warm_p95_ms=float(np.percentile(timings[1:],95)*1000),model_samples=len(timings),
                excludes=['YOLO','camera transport','IK','trajectory','GPU collision'],hardware_motion=False)
    args.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='observed_model'}))


if __name__=='__main__':main()
