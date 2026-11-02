#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/dds_profile.env"
export LD_LIBRARY_PATH="/opt/opencv-cuda-4.12.0/lib:${LD_LIBRARY_PATH:-}"
# These ROS adapters multiply small matrices. Large BLAS pools starve lidar
# callbacks on the Jetson; this changes scheduling, not sensor geometry.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ ! -f "$DDS_RMW_LIBRARY" ]]; then
  echo "DDS implementation not installed: $RMW_IMPLEMENTATION" >&2
  exit 1
fi
exec bash /home/whill/eye_ctrl_wheelchair/sensor_visualization/start_sensors.sh "$@"
