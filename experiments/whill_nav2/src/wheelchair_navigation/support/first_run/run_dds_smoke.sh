#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
receiver=''
cleanup() { if [[ -n "$receiver" ]]; then kill "$receiver" 2>/dev/null || true; wait "$receiver" 2>/dev/null || true; fi; }
trap cleanup EXIT
echo 'Fast DDS isolated smoke test'
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=199
python3 "$HERE/dds_smoke.py" receive & receiver=$!
sleep 1
python3 "$HERE/dds_smoke.py" send
wait "$receiver"
receiver=''
echo 'Cyclone DDS isolated smoke test'
ROOT=/home/whill/Shaki_ws/dds_eval_20261030/stage/opt/ros/humble
export LD_LIBRARY_PATH="$ROOT/lib:$ROOT/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=200
python3 "$HERE/dds_smoke.py" receive & receiver=$!
sleep 1
python3 "$HERE/dds_smoke.py" send
wait "$receiver"
receiver=''
