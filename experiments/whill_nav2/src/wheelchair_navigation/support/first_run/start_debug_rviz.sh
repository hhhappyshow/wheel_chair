#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/dds_profile.env"
source /opt/ros/humble/setup.bash
exec rviz2 -d "$HERE/sensors_debug_base.rviz"
