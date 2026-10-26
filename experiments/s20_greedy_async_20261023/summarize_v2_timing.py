"""Measured planning accounting for one saved online run; no diary writes."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np


def main():
    ap=argparse.ArgumentParser();ap.add_argument("--run-folder",type=Path,required=True);args=ap.parse_args();folder=args.run_folder
    data=json.loads((folder/"result.json").read_text());trajectory=np.load(folder/"trajectory.npz")
    rows=[];totals=defaultdict(float)
    for cycle in data["cycles"]:
        ident=f"B{cycle['arrival']}";events=[e for e in data["events"] if e["stage"].startswith(ident+"_")]
        groups=defaultdict(float)
        for e in events:
            stage=e["stage"];dt=e.get("wall_s",0)+e.get("seed_ik_s",0)+e.get("cartesian_repair_s",0)
            group="loaded_entrance_place" if "_candidate" in stage else "empty_retreat_return" if any(k in stage for k in ("_separate","_empty_exit","_outside_return","_return_observe")) else "view_pick_lift_transfer"
            groups[group]+=dt
            groups["cad_validation_s"]+=e.get("cad_validation_s",0)
        observations=[json.loads(p.read_text()) for p in (folder/"perception"/ident).glob("observed_box_view*.json")]
        sensor=sum(o.get("sensor_s",0) for o in observations)
        fitting=sum(o.get("model_s",o.get("modeling_s",0)) for o in observations)
        row=dict(id=ident,status=cycle["status"],space_planning_ms=cycle.get("space_planning_s",0)*1000,
                 trajectory_all_stages_s=sum(e.get("wall_s",0)+e.get("seed_ik_s",0)+e.get("cartesian_repair_s",0) for e in events),
                 **groups,sensor_s=sensor,fitting_s=fitting,
                 observation_including_view_motion_s=cycle.get("observation_and_model_s"),
                 async_wait_s=cycle.get("async_wait_s"),
                 overlap_window_s=cycle.get("pickup_transfer_overlap_window_s"),
                 query_attempts=sum("success" in e and "cache" not in e for e in events),
                 candidate_failures=len(cycle.get("candidate_failures",[])),
                 successful_repair_queries_including_prior_attempts=sum("Newton Cartesian IK" in e.get("engine","") for e in events),
                 reference_motion_s=float(np.sum(np.char.startswith(trajectory["stage"],ident+"_"))*.02))
        if "resume" in data and cycle["arrival"]>data["resume"]["completed_prefix"]:
            parent=json.loads((Path(data["resume"]["source"])/"result.json").read_text())
            suffix=[e for e in data["events"][len(parent["events"]):] if e["stage"].startswith(ident+"_")]
            row["last_resume_planning_s"]=sum(e.get("wall_s",0)+e.get("seed_ik_s",0)+e.get("cartesian_repair_s",0) for e in suffix)
        rows.append(row)
        for k,v in row.items():
            if isinstance(v,(int,float)):totals[k]+=v
    report=dict(completed=data["completed"],placed=data["placed"],rows=rows,totals=dict(totals),
        initialization_s=data["initialization_s"],total_wall_s=data["wall_s"],
        note="Reference motion is prescribed FK, not measured physical execution. Resumed item totals retain failed parent-checkpoint attempts; repair counts include those prior attempts, not only the final executed path. Cached successes have zero solve time. Observation span overlaps view query time and is not additive. Repair checks overlap CAD-validation timing, so those columns must not be added.")
    (folder/"timing_summary.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=="__main__":main()
