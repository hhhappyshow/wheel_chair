# CPS Inyern: S20 Online Pick and Place

Private source snapshot of the current S20 integration, exported on 2026-10-26.
This is not a copy of the whole CPS Intern workspace and not a standalone hardware-ready installer.

## Current Flow

Real RGB-D capture -> YOLO11m segmentation -> measured top plane and centered suction
-> asynchronous greedy placement and IK/CAD planning -> taught lift and entry
-> insert in depth, shift laterally, lower -> release confirmation
-> vertical 20cm retreat -> independent empty return.

- Incoming items are processed online; the next item is not supplied to the planner.
- Prefer horizontal placement in the same layer/depth region, then right-to-left; keep 90-degree alternatives.
- XY collision/layout envelope adds 5cm per side; measured height and grasp center are unchanged.
- Support is evaluated on unpadded physical footprints, not virtual margin material.
- Native SDK MoveL/MovePathJ is the current executor. Operator confirms suction/release; DO is not pressure feedback.
- This current baseline is heuristic/numerical planning, not a trained RL policy or a complete GSIP/KCSVSDF reproduction.

## Entry Points

In the original configured Ubuntu environment:

```bash
./s20 config
./s20 run --help
./s20 plan --observation /absolute/path/to/recorded/observation
./s20 run --execute
```

`run --execute` commands real hardware and retains operator confirmations. Do not run
against a different installation without its calibration, SDK and configuration.
New logs go to `experiments/s20_online_mixed8_20261024/runs/`.
Historical long commands remain compatible.

## Source Organization

- `deployment/`: host session, camera/observation adapters, configuration, pick/place preparation and native SDK bridge.
- `perception/`: object segmentation, depth/top-plane geometry and tool-envelope grasp selection.
- `straight_place/`: sparse candidates, ordering, IK/CAD and loaded/empty route preparation.
- `experiments/s20_greedy_async_20261023/`: inherited project planning and simulation integration dependencies.
- `records/S20/README.md`: detailed module map and evidence links in the original workspace.
- `records/S20/RESEARCH_DIARY.md`: dated successes, failures and evidence limitations.
- `source_manifest.json`: exact exported files and pinned external repositories.

Some evidence links refer to the original local workspace; raw captures, runs, videos
and old 32/41-box experiments are intentionally not included in this first source commit.

## Dependencies and Attribution

The code includes project-specific integration and modifications of inherited work.
It is not claimed to be entirely original code written by one author. See
[THIRD_PARTY.md](THIRD_PARTY.md) before redistribution.

External requirements include the pinned colleague robot/model repository, NVIDIA cuRobo,
Newton 1.3 runtime, Warp, PyTorch, NumPy, SciPy, OpenCV, python-fcl, PyYAML and Ultralytics.
The local planner interpreter historically lives in
`reproductions/nvidia_curobo_v2_20261023/.venv/bin/python`; that path name does not mean
the current LM planning path invokes cuRobo for every query. The runtime13 directory
is a separately installed Newton package, not project-authored source.

Jetson additionally requires ROS Humble, the installed RealSense DDS build and the
existing robot controller SDK. Camera weights, vendor binaries and virtual environments
are excluded. Install YOLO11m-seg weights from Ultralytics into
`experiments/s20_online_mixed8_20261024/perception/weights/`.

Current calibration and teaching configurations describe one installation; they are
not universal robot settings. Full independent installation has not been verified.
Recent four-recorded-item offline preparations passed; that is not evidence that the
latest configuration has completed four new hardware cycles or universal collision safety.
