"""Incremental geometry experiment: rear/right first, no saved slot order."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import time

import numpy as np
import yaml
from compare import Box,SCENE,p
from region_search import region_candidates,greedy_key


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--count",type=int,default=8)
    parser.add_argument("--compact-prior",action="store_true")
    parser.add_argument("--profiles",nargs="+",default=["depth","flat","pair","stack"])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    for name in ("greedy_frontier.py","region_search.py","compare.py","compact_prior.py"):
        shutil.copyfile(Path(__file__).with_name(name),args.output/("source_"+name))
    shutil.copyfile(SCENE,args.output/"scene_tf.yaml")
    cfg=yaml.safe_load(SCENE.read_text());hull=p.Hull.from_config(cfg)
    from compact_prior import CompactPrior
    compact=CompactPrior(cfg,hull)
    cases=[]
    for profile in args.profiles:
        placed=[];steps=[]
        for arrival in range(args.count):
            # No future objects, final layout or domain witnesses are input.
            dims=(.55,.40,.25);start=time.perf_counter()
            rows,stats=compact.candidates(dims,placed,profile) if args.compact_prior else region_candidates(dims,placed,hull,profile=profile)
            selected=None;checks=0
            for b in rows:
                checks+=1;route=p.route_to(b,placed,hull)
                if route is not None:selected=b;break
            elapsed=time.perf_counter()-start
            step=dict(arrival=arrival+1,prefix=len(placed),box=asdict(selected) if selected else None,
                      planning_s=elapsed,route_checks=checks,candidates=stats,
                      route=route if selected else None,
                      best_geometric_key=(compact.key(rows[0],placed,profile) if args.compact_prior else greedy_key(rows[0],placed,hull,profile)) if rows else None)
            if selected:
                step["box"]["id"]=f"B{arrival+1}";placed.append(Box(**step["box"]))
            steps.append(step)
        times=[s["planning_s"] for s in steps]
        cases.append(dict(profile=profile,count=len(placed),steps=steps,
                          median_ms=float(np.median(times)*1000),max_ms=max(times)*1000))
    result=dict(scope="Online geometry + swept cargo doorway test ONLY; no arm IK/pick/release physics",hull=hull.as_dict(),
                axes="x=0 door, +x rear; y=0 straight/right wall viewed into door; +y left/chamfer",cases=cases,
                historical_endpoints_used=False,future_arrivals_used=False,hardware_motion=False)
    (args.output/"result.json").write_text(json.dumps(result,indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig,axes=plt.subplots(2,len(cases),figsize=(max(5,3.75*len(cases)),7),layout="constrained",squeeze=False)
    for j,case in enumerate(cases):
        for row in (0,1):
            projections={}
            for s in case["steps"][:3]:
                if not s["box"]:continue
                x,y,z=s["box"]["pos"];lx,ly,lz=s["box"]["dims"]
                rect=(x,y,lx,ly) if row==0 else (y,z,ly,lz)
                projections.setdefault(rect,[]).append(s["arrival"])
            ax=axes[row,j]
            for (u,v,du,dv),ids in projections.items():
                color=plt.get_cmap("tab10")((ids[0]-1)%10)
                ax.add_patch(Rectangle((u,v),du,dv,facecolor=color,edgecolor="white",alpha=.65))
                ax.text(u+du/2,v+dv/2,",".join(map(str,ids)),ha="center",va="center",fontsize=9)
        axes[0,j].set_title(case["profile"]+f" | {case['count']}/{args.count}")
        for ax in axes[:,j]:ax.grid(alpha=.15)
        axes[0,j].set_xlim(0,hull.length);axes[0,j].set_xlabel("Door -> rear X (m)")
        axes[0,j].set_ylim(0,hull.width);axes[0,j].set_ylabel("Right wall -> left Y (m)")
        axes[1,j].set_xlim(0,hull.width);axes[1,j].set_xlabel("Right wall -> left Y (m)")
        axes[1,j].set_ylim(0,hull.height);axes[1,j].set_ylabel("Height Z (m)")
    fig.suptitle("First 3 online arrivals\nGeometry only; counts cover full stream")
    fig.savefig(args.output/"layouts.png",dpi=145);plt.close(fig)
    print(json.dumps([{k:c[k] for k in ("profile","count","median_ms","max_ms")} for c in cases]),flush=True)


if __name__=="__main__":main()
