"""Compare native Newton GPU contacts and CPU FCL on the SAME saved CAD poses.

This is an adapter/measurement, not continuous collision certification.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from compare import ROOT, UPSTREAM, SCENE, p
import yaml
from online_packing_newton import make_model
from demo_collision import surface_gaps
from newton_compare import Collision
import newton
import warp as wp


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-folder", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=10)
    args=parser.parse_args()
    wp.set_device("cuda:0")
    data=np.load(args.run_folder/"trajectory.npz")
    sample=dict(container=p.Hull.from_config(yaml.safe_load(SCENE.read_text())).as_dict(),
                steps=[dict(incoming=dict(id=f"B{i+1}",dims=[.55,.4,.25])) for i in range(3)])
    model,box_ids,_,_=make_model(sample,UPSTREAM/"policy/vendor/roboticarm",SCENE)
    state=model.state();boxes=list(box_ids.values());tcp=model.body_label.index("S20/suction_contact_frame")
    w=SimpleNamespace(model=model,state=state,fk=model.state(),boxes=boxes,q=model.joint_q.numpy(),
                      qidx=np.arange(6),arm_count=min(boxes),tcp=tcp,held=None,
                      grip_offset=np.array([0,0,.125]),grip_rot=R.from_euler("x",np.pi))
    cpu=Collision(w)
    flags=model.shape_flags.numpy()
    for shape in list(cpu.objs):
        body=cpu.bodies[shape]
        if body>=0 and model.body_label[body].endswith("/payload_box"):
            del cpu.objs[shape];cpu.arm.discard(shape);flags[shape]&=~2
    model.shape_flags.assign(flags)
    contacts=model.contacts()
    # Original source adjacency filters and attachment exclusions match FCL.
    def gpu_query():
        model.collide(state,contacts)
        n=int(contacts.rigid_contact_count.numpy()[0])
        if not n:return []
        s0=contacts.rigid_contact_shape0.numpy()[:n];s1=contacts.rigid_contact_shape1.numpy()[:n]
        gaps=surface_gaps(state.body_q.numpy(),cpu.bodies[s0],cpu.bodies[s1],
                          contacts.rigid_contact_point0.numpy()[:n],contacts.rigid_contact_point1.numpy()[:n],
                          contacts.rigid_contact_normal.numpy()[:n],contacts.rigid_contact_margin0.numpy()[:n],
                          contacts.rigid_contact_margin1.numpy()[:n])
        hits=[]
        active={boxes[i] for i in cpu.arrived}|({w.held} if w.held is not None else set())
        for a,b,gap in zip(s0,s1,gaps):
            a,b=int(a),int(b);ba,bb=int(cpu.bodies[a]),int(cpu.bodies[b])
            if gap>=0 or a not in cpu.objs or b not in cpu.objs:continue
            if (ba in boxes and ba not in active) or (bb in boxes and bb not in active):continue
            if ba==bb or (ba>=0 and bb>=0 and cpu.rigid[ba]==cpu.rigid[bb]) or tuple(sorted((a,b))) in cpu.filters:continue
            if not(a in cpu.arm or b in cpu.arm or ba==w.held or bb==w.held):continue
            if (a in cpu.base and model.shape_label[b]=="robot_pedestal") or (b in cpu.base and model.shape_label[a]=="robot_pedestal"):continue
            if (a in cpu.suction and bb==w.held) or (b in cpu.suction and ba==w.held):continue
            hits.append(dict(shapes=[model.shape_label[a],model.shape_label[b]],gap_m=float(gap)))
        return hits

    records=[];startup=time.perf_counter();gpu_query();wp.synchronize();startup=time.perf_counter()-startup
    indices=sorted(set(range(0,len(data["joint_q"]),args.stride))|{len(data["joint_q"])-1})
    rng=np.random.default_rng(23)
    for index in indices:
        stage=str(data["stage"][index]);arrival=int(stage.split("_")[0][1:])-1
        loaded="insert" in stage
        cpu.arrived=set(range(arrival+(not loaded)));cpu.current_source=arrival if loaded else None
        w.held=boxes[arrival] if loaded else None
        # Perturbed configurations deliberately include additional colliding
        # states, rather than comparing only already-accepted references.
        for perturb in (False,True):
            qi=data["joint_q"][index].copy()
            if perturb:qi+=rng.normal(0,.20,6)
            w.q[:6]=qi;state.joint_q.assign(w.q)
            newton.eval_fk(model,state.joint_q,state.joint_qd,state)
            poses=state.body_q.numpy();poses[boxes]=data["body_q"][index,boxes]
            if loaded:
                rot=R.from_quat(poses[tcp,3:]);poses[w.held]=np.r_[poses[tcp,:3]+rot.apply(w.grip_offset),(rot*w.grip_rot).as_quat()]
            state.body_q.assign(poses)
            begin=time.perf_counter();safe=cpu.validate(qi,actual=True);cpu_s=time.perf_counter()-begin
            cpu_hits=list(cpu.last_hits)
            begin=time.perf_counter();gpu_hits=gpu_query();gpu_s=time.perf_counter()-begin
            records.append(dict(index=index,stage=stage,perturbed=perturb,cpu_collision=not safe,gpu_collision=bool(gpu_hits),cpu_s=cpu_s,gpu_s=gpu_s,cpu_hits=cpu_hits,gpu_hits=gpu_hits))
    report=dict(scope=__doc__,backend="Newton1.3/Warp1.18 CUDA native contact pipeline",startup_s=startup,
                samples=len(records),cpu_collision=sum(x["cpu_collision"] for x in records),gpu_collision=sum(x["gpu_collision"] for x in records),
                disagreements=sum(x["cpu_collision"]!=x["gpu_collision"] for x in records),
                cpu_total_s=sum(x["cpu_s"] for x in records),gpu_total_s=sum(x["gpu_s"] for x in records),
                records=records,geometry="same imported CAD, native contact generation vs FCL triangle intersection; not assumed equivalent")
    (args.run_folder/"gpu_contacts.json").write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!="records"}))


if __name__=="__main__":main()
