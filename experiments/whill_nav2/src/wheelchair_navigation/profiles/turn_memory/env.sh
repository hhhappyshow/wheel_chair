#!/usr/bin/env bash
root=/home/whill/Shaki_ws/nav2_humble_update_20261031
export WHEELCHAIR_GAZEBO_RUNTIME="$root/sim_runtime"
source /home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/scripts/env.sh
source /home/whill/Shaki_ws/src/WheelChair_ws/install/local_setup.bash
source "$root/install/local_setup.bash"
source "$root/rotation_shim_20261101/install/local_setup.bash"
source "$root/turn_memory_20261101/shim_decel_ws/install/local_setup.bash"
turn_memory_vendor="$root/turn_memory_20261101/vendor/prefix/opt/ros/humble"
export AMENT_PREFIX_PATH="$turn_memory_vendor:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$turn_memory_vendor/lib:${LD_LIBRARY_PATH:-}"
