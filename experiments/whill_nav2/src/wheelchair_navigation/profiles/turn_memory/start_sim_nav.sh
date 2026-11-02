#!/usr/bin/env bash
set -eo pipefail
root=/home/whill/Shaki_ws/nav2_humble_update_20261031
source "$root/turn_memory_20261101/env.sh"
source "$root/simulation_dds.env"
[[ "$ROS_DOMAIN_ID" == 198 && "$ROS_LOCALHOST_ONLY" == 1 ]]
profile=${TURN_PROFILE:-simulation.yaml}
case "$profile" in
  simulation.yaml|simulation_unknown.yaml) ;;
  simulation_rotation_smooth.yaml|simulation_angular_accel_1.yaml|simulation_angular_accel_08.yaml) source "$root/turn_memory_20261101/shim_decel_ws/install/local_setup.bash" ;;
  *) exit 2 ;;
esac
exec ros2 launch "$root/turn_memory_20261101/navigation.launch.py" \
  params_file:="$root/turn_memory_20261101/$profile" \
  use_sim_time:=true start_map_adapter:=false
