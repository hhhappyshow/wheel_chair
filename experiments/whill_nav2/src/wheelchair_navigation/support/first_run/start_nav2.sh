#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/whill/Shaki_ws/src/WheelChair_ws/install/local_setup.bash
source /home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
test "$(ros2 pkg prefix nav2_controller)" = /home/whill/Shaki_ws/src/WheelChair_ws/install/nav2_controller
exec ros2 launch /home/whill/eye_ctrl_wheelchair/wheelchair_shared_ws/src/wheelchair_nav2_pipeline/launch/navigation.launch.py \
  params_file:=/home/whill/Shaki_ws/src/WheelChair_ws/first_run/nav2_low_speed.yaml \
  start_map_adapter:=true connect_actuator:=false start_domain_preview:=false rviz:=false
