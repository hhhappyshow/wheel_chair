#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/dds_profile.env"
if [[ ! -f "$DDS_RMW_LIBRARY" ]]; then
  echo "DDS implementation not installed: $RMW_IMPLEMENTATION" >&2
  exit 1
fi
for file in "$HERE/nav2_bidirectional_full.yaml" "$HERE/nav2_goal.rviz" \
  /opt/ros/humble/setup.bash /home/whill/Shaki_ws/src/WheelChair_ws/install/local_setup.bash \
  /home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/install/local_setup.bash \
  /home/whill/Shaki_ws/esp32_velocity_interface/ros2_ws/install/local_setup.bash; do
  [[ -f "$file" ]] || { echo "Missing $file" >&2; exit 1; }
done
if [[ ${1:-} == --check ]]; then
  echo "Files present; DDS=$RMW_IMPLEMENTATION; no nodes started."
  exit 0
fi
[[ $# == 0 ]] || { echo "Usage: $0 [--check]" >&2; exit 2; }
[[ -n ${DISPLAY:-} || -n ${WAYLAND_DISPLAY:-} ]] || { echo 'Run in the Jetson graphical desktop.' >&2; exit 1; }
if pgrep -f '/nav2_planner/lib/nav2_planner/planner_server( |$)' >/dev/null || \
   pgrep -f '/nav2_controller/lib/nav2_controller/controller_server( |$)' >/dev/null; then
  echo 'A Nav2 planner is already running. Stop the previous navigation stack first.' >&2; exit 1
fi
[[ -e /dev/ble2wchair ]] || { echo 'ESP32 serial device is missing.' >&2; exit 1; }
if fuser /dev/ble2wchair >/dev/null 2>&1; then
  echo 'ESP32 serial device is already in use. Stop its previous control window first.' >&2; exit 1
fi
source /opt/ros/humble/setup.bash
source /home/whill/Shaki_ws/src/WheelChair_ws/install/local_setup.bash
source /home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/install/local_setup.bash
[[ "$(ros2 pkg prefix nav2_controller)" == /home/whill/Shaki_ws/src/WheelChair_ws/install/nav2_controller ]] || {
  echo 'Wrong Nav2 overlay selected.' >&2; exit 1;
}
python3 "$HERE/check_sensor_ready.py"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
ros2 launch /home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/wheelchair_nav2_pipeline/launch/navigation.launch.py \
  params_file:="$HERE/nav2_bidirectional_full.yaml" start_map_adapter:=true connect_actuator:=false \
  start_domain_preview:=false rviz:=false &
pids+=("$!")
python3 "$HERE/check_nav2_ready.py"
bash /home/whill/Shaki_ws/esp32_velocity_interface/scripts/run.sh \
  hardware_enabled:=true cmd_vel_topic:=/wheelchair/nav2/cmd_vel allow_reverse:=true &
pids+=("$!")
bash "$HERE/start_rviz.sh" &
pids+=("$!")
echo 'Nav2, ESP32 and RViz started. Arm in the ESP32 control window.'
wait -n "${pids[@]}"
echo 'A navigation component exited; stopping this launcher.' >&2
exit 1
