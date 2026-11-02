#!/usr/bin/env bash
set -eo pipefail
root=/home/whill/Shaki_ws/nav2_humble_update_20261031
run="$root/${BUILD_RUN_ID:-build_resume_20261031_1200}"
mkdir "$run"
exec > >(tee -a "$run/build.log") 2>&1
trap 'status=$?; printf "%s\n" "$status" > "$run/exit_status"; exit "$status"' EXIT
cd "$root"
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j2
export CMAKE_BUILD_PARALLEL_LEVEL=2
printf 'start=%s\nsource=' "$(date -Is)"
git -C src/navigation2 rev-parse HEAD
colcon build --executor sequential --packages-up-to \
  nav2_controller nav2_mppi_controller nav2_smac_planner nav2_planner \
  nav2_bt_navigator nav2_behaviors nav2_smoother nav2_velocity_smoother \
  nav2_lifecycle_manager nav2_waypoint_follower nav2_collision_monitor \
  nav2_rviz_plugins --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
printf 'end=%s\n' "$(date -Is)"
