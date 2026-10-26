"""Inspect recorded FK frames with original CAD and the original LiDAR visual box.

This is discrete collision evidence only, not a continuous or dynamics proof.
"""
import argparse
import json
from pathlib import Path
import re
import sys
import time
from types import SimpleNamespace

import numpy as np
import yaml
from compare import ROOT, UPSTREAM, p
from online_packing_newton import make_model
import fcl
import warp as wp
sys.path.insert(0,str(ROOT/"experiments/s20_newton_fullcycle20_20261021"))
from collision import Collision


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-folder",type=Path,required=True)
    args=parser.parse_args();out=args.run_folder;started=time.perf_counter()
    report=json.loads((out/"result.json").read_text());scene=out/"scene_tf.yaml"
    h=p.Hull.from_config(yaml.safe_load(scene.read_text()))
    sample=dict(container=h.as_dict(),steps=[dict(incoming=dict(id=f"B{i+1}",dims=d)) for i,d in enumerate(report["source_catalog_m"])])
    wp.set_device("cuda:0")
    model,ids,_,_=make_model(sample,UPSTREAM/"policy/vendor/roboticarm",scene)
    flags=model.shape_flags.numpy();bodies=model.shape_body.numpy();added=[]
    for shape,body in enumerate(bodies):
        name=model.body_label[body] if body>=0 else ""
        if name.endswith("/payload_box"):flags[shape]=0
        if name.endswith("/mid360_mount_frame"):
            flags[shape]|=2;added.append(dict(shape=model.shape_label[shape],dimensions_m=(2*model.shape_scale.numpy()[shape]).tolist()))
    model.shape_flags.assign(flags);state=model.state()
    world=SimpleNamespace(model=model,state=state,fk=model.state(),boxes=list(ids.values()),
        q=model.joint_q.numpy().copy(),qidx=np.arange(6),tcp=model.body_label.index("S20/suction_contact_frame"),
        arm_count=min(ids.values()),held=None,grip_offset=np.zeros(3),grip_rot=None)
    cad=Collision(world);data=np.load(out/"trajectory.npz")
    if list(data["body_labels"])!=model.body_label:raise ValueError("Different recorded model order")
    failures=[];rejected=0;admitted=0;max_contact=0.
    for index,(poses,q,stage) in enumerate(zip(data["body_q"],data["joint_q"],data["stage"])):
        state.body_q.assign(poses);world.q[:6]=q
        match=re.match(r"B(\d+)_",str(stage));current=int(match[1])-1 if match else 0
        cad.arrived={i for i,body in enumerate(world.boxes) if np.linalg.norm(poses[body,:3])<20}
        cad.current_source=None if any(s in stage for s in ("empty_exit","outside_return","return_observe")) else current
        held=any(s in stage for s in ("ideal_vacuum_on","_lift","raise_above_base","outside_transfer","insert_leg"))
        world.held=world.boxes[current] if held else None
        original=cad.filters.copy();bad=None
        try:
            while not cad.validate(q,actual=True):
                pair=cad.last_hits[0];cargo=[s for s in pair if s.startswith("B") and s.endswith("_collision_box")]
                allow="insert_leg" in stage and cargo and all(s in cargo or s=="container_collision_full" for s in pair)
                indices=[model.shape_label.index(s) for s in pair]
                contact=fcl.CollisionResult()
                fcl.collide(cad.objs[indices[0]],cad.objs[indices[1]],fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),contact)
                depth=max((c.penetration_depth for c in contact.contacts),default=float("inf"))
                if allow and depth<=.001:
                    admitted+=1;max_contact=max(max_contact,float(depth));cad.filters.add(tuple(sorted(indices)));continue
                bad=dict(frame=index,stage=str(stage),pairs=pair,depth_m=float(depth));break
        finally:cad.filters=original
        if bad:
            rejected+=1
            if len(failures)<30:failures.append(bad)
    result=dict(scope=__doc__,frames=len(data["body_q"]),rejected_frames=rejected,first_failures=failures,
        added_lidar_proxy=added,intended_cargo_contact_limit_m=.001,admitted_contact_pairs=admitted,
        maximum_admitted_depth_m=max_contact,wall_s=time.perf_counter()-started,
        continuous_validation=False,dynamic_execution=False)
    (out/"recorded_cad_with_lidar.json").write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)


if __name__=="__main__":main()
