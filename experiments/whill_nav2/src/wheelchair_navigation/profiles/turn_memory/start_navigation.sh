#!/usr/bin/env bash
set -eo pipefail
root=/home/whill/Shaki_ws/nav2_humble_update_20261031
here="$root/turn_memory_20261101"
first=/home/whill/Shaki_ws/src/WheelChair_ws/first_run
interface=/home/whill/Shaki_ws/esp32_velocity_interface
source "$here/env.sh"
source "$first/dds_profile.env"
[[ -f "$here/navigation.yaml" ]]
if [[ ${1:-} == --check ]]; then
  ros2 pkg prefix laser_filters
  ros2 pkg prefix nav2_rotation_shim_controller
  echo 'Turn/memory navigation profile present.'
  exit 0
fi
[[ $# == 0 && -n ${DISPLAY:-} ]]
if pgrep -f '/nav2_(planner|controller)/lib/nav2_(planner|controller)/(planner|controller)_server( |$)' >/dev/null; then
  echo 'Nav2 already running; stop the current instance before switching.' >&2; exit 1
fi
[[ -e /dev/ble2wchair ]]
if fuser /dev/ble2wchair >/dev/null 2>&1; then echo 'ESP32 interface already in use' >&2; exit 1; fi
python3 "$first/check_sensor_ready.py"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
ros2 launch "$here/navigation.launch.py" &
pids+=("$!")
python3 "$first/check_nav2_ready.py"
(
  source "$interface/ros2_ws/install/local_setup.bash"
  python3 "$interface/scripts/set_output_mode.py" live
  exec bash "$interface/scripts/run.sh" hardware_enabled:=true \
    cmd_vel_topic:=/wheelchair/nav2/cmd_vel allow_reverse:=true latched_arm:=true
) &
pids+=("$!")
bash "$first/start_rviz.sh" &
pids+=("$!")
echo 'Independent turn/memory navigation ready; local ARM required.'
wait -n "${pids[@]}"
exit 1
