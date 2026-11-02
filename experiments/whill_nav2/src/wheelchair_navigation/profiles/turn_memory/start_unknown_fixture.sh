#!/usr/bin/env bash
set -eo pipefail
root=/home/whill/Shaki_ws/nav2_humble_update_20261031
source "$root/turn_memory_20261101/env.sh"
source "$root/simulation_dds.env"
exec python3 "$root/turn_memory_20261101/unknown_fixture.py" --ros-args -p use_sim_time:=true
