# Source Attribution

This repository is an integration/research snapshot, not a blanket originality claim.
No new license is asserted over inherited code, robot CAD or vendor SDKs.

| Component | Origin | Treatment |
| --- | --- | --- |
| S20 online integration and experiments | This project's local experiments and Codex-assisted adaptations | Exported with original paths; inherited portions still require line-level provenance review |
| Colleague geometry/robot/model/trajectory helpers | https://github.com/ludan0414/RoboticArm | Not vendored; pinned commit in source_manifest.json |
| NVIDIA cuRobo | https://github.com/NVlabs/curobo | Not vendored; pinned commit in source_manifest.json |
| Newton/Warp | Newton and NVIDIA Warp upstream packages | External packages, not original research source |
| YOLO/Ultralytics | https://github.com/ultralytics/ultralytics | External dependency; upstream licensing applies; weights not uploaded |
| FCL/python-fcl and numerical libraries | Their respective upstream projects | External dependencies |
| tools/three_box_original | Inherited local three-box implementation | Preserved attribution status as inherited; individual authorship not fully verified |
| Robot controller SDK and RealSense DDS | Existing installed hardware dependencies | Vendor binaries not uploaded |
| WHILL Navigation2 | https://github.com/ros-navigation/navigation2 | Pinned upstream revision plus local patch; Apache-2.0 and upstream notices remain applicable |
| WHILL RotationShim overlays | Nav2 package with project integration changes | Existing package notices retained; no blanket relicensing |
| WHILL ESP32/controller integration | Existing whill_controller package and project adaptations | Original maintainer/license metadata retained; shared optional UI changes documented |

The project uses the colleague model repository at its original relative location.
Obtain it independently and honor its original notices; do not replace its license
with a project-wide license. Public release requires reviewing code/CAD rights and
installation-specific calibration and imagery. This first repository is private.
