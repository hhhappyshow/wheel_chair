#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/whill/Shaki_ws/src/WheelChair_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
exec /opt/ros/humble/lib/rviz2/rviz2 -d /home/whill/Shaki_ws/src/WheelChair_ws/first_run/nav2_goal.rviz
