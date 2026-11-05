# Source Attribution

This repository is an integration/research snapshot, not a blanket originality claim.
No new license is asserted over inherited code or vendor SDKs.

| Component | Origin | Treatment |
| --- | --- | --- |
| WHILL navigation integration and experiments | This project's local experiments and adaptations | Exported with original paths; inherited portions still require line-level provenance review |
| WHILL Navigation2 | https://github.com/ros-navigation/navigation2 | Pinned upstream revision plus local patch; Apache-2.0 and upstream notices remain applicable |
| WHILL RotationShim overlays | Nav2 package with project integration changes | Existing package notices retained; no blanket relicensing |
| WHILL ESP32/controller integration | Existing whill_controller package and project adaptations | Original maintainer/license metadata retained; shared optional UI changes documented |
| Robot sensors (MID360, Odin) and vendor drivers | Existing installed hardware dependencies | Vendor binaries not uploaded |

Pinned upstream revisions are recorded in `experiments/whill_nav2/upstream/navigation2.repos`.
Public release requires reviewing code rights and installation-specific calibration and
imagery. This repository is private.
