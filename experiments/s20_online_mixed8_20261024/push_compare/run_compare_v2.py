"""CPU paired eight-arrival v2 run; exact support, no future or dynamics."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[name] = "1"
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from mixed_arrivals import ArrivalStream
from local_rearrange_v2 import StrictCompactPrior, metrics, plan_current
from compare import SCENE, p
import yaml


def run(seed, count, push, hull, cfg):
    stream = ArrivalStream(seed=seed)
    boxes = []; steps = []; prior = StrictCompactPrior(cfg, hull)
    for i in range(count):
        incoming = stream.get(i)
        before = [asdict(b) for b in boxes]
        before_metrics = metrics(boxes, prior)
        started = time.perf_counter()
        proposal = plan_current(incoming["dimensions_m"], boxes, hull, cfg,
                                allow_push=push, label=f"B{i+1}")
        assert before == [asdict(b) for b in boxes]
        after_push = proposal["proposed_placed"]
        selected = proposal["candidate"]
        committed = selected is not None
        if committed:
            boxes = list(after_push)+[selected]
            if proposal["selected_trial"] is not None:
                proposal["trials"][proposal["selected_trial"]]["committed"] = True
        else:
            assert proposal["movement"] is None
        step = dict(incoming=incoming, occupied_before=before,
                    occupied_after_push=[asdict(b) for b in after_push],
                    metrics_before_push=before_metrics,
                    movement=proposal["movement"], current_choice=asdict(selected) if selected else None,
                    current_route=proposal["route"], direct_search=proposal["direct_search"],
                    baseline_candidate=proposal["baseline_candidate"],
                    baseline_metrics_with_current=proposal["baseline_metrics_with_current"],
                    trials=proposal["trials"], metrics_after_push=metrics(after_push, prior),
                    metrics_after_arrival=metrics(boxes, prior), elapsed_s=time.perf_counter()-started,
                    future_arrivals_used=0, rejection_reason=None if committed else "no_strictly_supported_routed_candidate")
        steps.append(step)
        print(json.dumps(dict(mode="push" if push else "no_push", index=i, size_inches=incoming["size_inches"],
                              dimensions_m=incoming["dimensions_m"], movement=step["movement"],
                              occupied_before=before, occupied_after_push=step["occupied_after_push"],
                              current_choice=step["current_choice"], accepted=len(boxes),
                              elapsed_s=step["elapsed_s"])), flush=True)
    return dict(mode="push" if push else "no_push", steps=steps, final_boxes=[asdict(b) for b in boxes],
                metrics=metrics(boxes, prior), total_planning_s=sum(s["elapsed_s"] for s in steps),
                committed_pushes=sum(s["movement"] is not None for s in steps),
                arrival_state_after_run=stream.state())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=73)
    parser.add_argument("--count", type=int, default=8)
    parser.add_argument("--output", type=Path, default=HERE/"result_seed73_v2.json")
    args = parser.parse_args()
    if args.output.resolve().parent != HERE or args.output.exists():
        parser.error("Choose a new result filename inside push_compare")
    cfg = yaml.safe_load(SCENE.read_text()); hull = p.Hull.from_config(cfg)
    runs = [run(args.seed, args.count, push, hull, cfg) for push in (False, True)]
    assert runs[0]["arrival_state_after_run"] == runs[1]["arrival_state_after_run"]
    a, b = [r["metrics"] for r in runs]
    summary = dict(size_order=[s["incoming"]["size_inches"] for s in runs[0]["steps"]],
                   paired_full_arrivals_and_rng_states_equal=True,
                   accepted={r["mode"]: r["metrics"]["count"] for r in runs},
                   committed_pushes=runs[1]["committed_pushes"],
                   count_gain=b["count"]-a["count"], volume_gain_m3=b["packed_volume_m3"]-a["packed_volume_m3"],
                   envelope_utilization={r["mode"]: r["metrics"]["envelope_utilization"] for r in runs},
                   min_exact_support_fraction={r["mode"]: r["metrics"].get("minimum_exact_support_fraction") for r in runs},
                   planning_s={r["mode"]: r["total_planning_s"] for r in runs},
                   push_rejection_counts=dict(Counter(t["rejection_reason"] for s in runs[1]["steps"]
                                                      for t in s["trials"] if t["rejection_reason"])))
    report = dict(version=2, seed=args.seed, count=args.count, summary=summary, runs=runs,
                  scene_config=str(SCENE), scene_config_snapshot=cfg, hull=hull.as_dict(),
                  arrival_protocol="Parent mixed_arrivals.ArrivalStream; independent identical seed instances; get(i) only when current; no runtime or future list",
                  support="Both arms: floor or actual coplanar contact union >=80% footprint AND center projection in contact union; height tolerance1e-7m; no 5cm relaxation",
                  move_gate="Single horizontal <=5cm slide; existing upper boxes frozen; validate >=80% and center support for every box at all edge/center breakpoints and interval midpoints; swept collision checked; with same acceptance never enlarge current-prefix envelope; require smaller envelope or greater exact contact",
                  assumption="Commanded displacement equals actual displacement; no friction, force, contact dynamics, arm/tool access or IK certificate",
                  future_arrivals_used=0, gpu_used=False, robot_execution=False)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
