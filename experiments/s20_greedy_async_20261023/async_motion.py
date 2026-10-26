"""Overlap a fresh single-box joint-plan query with paced Newton Pick replay.

Reference-only prefetch. Saved Pick replay is kinematic, not physical Pick.
The returned entry reference still needs its actual staging connector.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
from compare import ROOT, UPSTREAM, SCENE
from online_packing_newton import make_model
from online_packing_real import Hull
import yaml
import newton
import warp as wp


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--mode",choices=("async","serial"),default="async")
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    wp.set_device("cuda:0")
    bundle=UPSTREAM/"policy/demo_data/site_four_colors_right160_observe120"
    inputs=json.loads((bundle/"input.json").read_text())
    cfg=yaml.safe_load(SCENE.read_text());hull=Hull.from_config(cfg)
    sample=dict(container=hull.as_dict(),steps=[dict(incoming=dict(id=f"B{i+1}",dims=d)) for i,d in enumerate(inputs["boxes"])])
    model,_,_,_=make_model(sample,UPSTREAM/"policy/vendor/roboticarm",SCENE)
    data=np.load(bundle/"trajectory.npz");state=model.state()
    # Ground pickup front-half only, before saved insertion/place.
    stages=data["stage"];ids=data["box_id"]
    candidates=np.flatnonzero((ids=="B2") & np.isin(stages,["APPROACH","PICK","ATTACH","LIFT"]))
    if not len(candidates):raise ValueError("No recorded pick front-half")
    start=time.perf_counter()
    logfile=(args.output/"planner.log").open("w")
    cmd=[str(ROOT/"reproductions/newton_box_20261019/.venv/bin/python"),
         str(Path(__file__).with_name("newton_compare.py")),"--output",str(args.output/"motion"),
         "--endpoint-first","--prior-first","--release-contact","--count","1"]
    process=subprocess.Popen(cmd,stdout=logfile,stderr=subprocess.STDOUT,env=os.environ.copy())
    if args.mode=="serial":process.wait()
    foreground_start=time.perf_counter()
    samples=[]
    for i,index in enumerate(np.linspace(candidates[0],candidates[-1],301).astype(int)):
        state.body_q.assign(data["body_q"][index]);state.joint_q.assign(data["joint_q"][index])
        samples.append(dict(frame=int(index),elapsed_s=time.perf_counter()-start,phase=str(stages[index])))
        time.sleep(max(0.,foreground_start+(i+1)*.02-time.perf_counter()))
    staged=time.perf_counter();code=process.wait();finished=time.perf_counter();logfile.close()
    if code:raise RuntimeError("Background plan failed; retained planner.log")
    plan=json.loads((args.output/"motion/result.json").read_text())
    # Backend wall_s ends before saving; output/status may take a little more.
    result=dict(scope=__doc__,mode=args.mode,motion_requested=False,requested_current_box_dims_m=[.55,.4,.25],
                foreground="301 saved Newton ground-Pick frames paced20ms; no dynamics or hardware",
                planned_objects=1,kinematic_motion_reference_passed=plan["placed"]==1,
                foreground_wall_s=staged-foreground_start,backend_work_wall_s=plan["wall_s"],
                total_async_wall_s=finished-start,wait_after_staging_s=finished-staged,
                nonoverlapped_estimate_s=(staged-foreground_start)+plan["wall_s"],
                estimate_scope="serial estimate adds measured backend work, not a paired serial rerun",
                reference_executable=False,actual_staging_connector_verified=False,
                frames=samples,command=cmd)
    (args.output/"result.json").write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ("frames","command")},indent=2))


if __name__=="__main__":main()
