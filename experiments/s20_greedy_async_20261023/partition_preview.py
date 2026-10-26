"""Current-arrival greedy zoning comparison; geometry only, no arm claim."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import yaml
from compare import SCENE, Box, p
from compact_prior import CompactPrior


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cfg=yaml.safe_load(SCENE.read_text());h=p.Hull.from_config(cfg);reports={}
    for name,catalog in (("pure20",[[.55,.40,.25]]*8),("mixed20_24_zoned",[[.55,.40,.25],[.65,.45,.30]]*4),("pure24",[[.65,.45,.30]]*8)):
        prior=CompactPrior(cfg,h,max_box_top_m=1.,depth_step_m=.1)
        prior.row_first=True;prior.horizontal_first=True
        placed=[];steps=[]
        for i,dims in enumerate(catalog):
            prior.lateral_bounds=(0.,h.flat_y/2) if name=="mixed20_24_zoned" and dims[0]<.6 else (h.flat_y/2,h.flat_y) if name=="mixed20_24_zoned" else None
            start=time.perf_counter();rows,stats=prior.candidates(dims,placed);selected=None
            for b in rows:
                route=p.route_to(b,placed,h,min_entry_z=.4)
                if route is not None:selected=b,route;break
            row=dict(id=f"B{i+1}",source_dimensions_m=dims,zone=prior.lateral_bounds,planning_s=time.perf_counter()-start,candidates=len(rows),occupied_before=[asdict(b) for b in placed])
            if selected is None:
                row["status"]="unresolved";steps.append(row);break
            b,route=selected;b=Box(f"B{i+1}",b.dims,b.pos);placed.append(b)
            row.update(status="geometric_route_accepted",box=asdict(b),route=route);steps.append(row)
        reports[name]=dict(scope=__doc__,accepted=len(placed),requested=8,steps=steps,source_catalog_m=catalog,
            hull=h.as_dict(),scene_config=str(SCENE),robot_execution=False,physics_execution=False,
            max_box_top_above_floor_m=1.,reach_radius_m=1.7,
            future_arrivals_used=False,partition="20 right half, 24 left half" if "zoned" in name else "no zoning")
    (args.output/"comparison.json").write_text(json.dumps(reports,indent=2))
    print(json.dumps({k:dict(accepted=v["accepted"],planning_ms=[round(s["planning_s"]*1000,3) for s in v["steps"]]) for k,v in reports.items()}))


if __name__=="__main__":main()
