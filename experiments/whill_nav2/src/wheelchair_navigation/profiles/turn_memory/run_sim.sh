#!/usr/bin/env bash
set -eo pipefail
root=/home/whill/Shaki_ws/nav2_humble_update_20261031
here="$root/turn_memory_20261101"
source "$here/env.sh"
export IGN_PARTITION=shaki_nav2_sim_20261031 SIM_RMW=rmw_cyclonedds_cpp
source "$root/simulation_dds.env"
[[ "$ROS_DOMAIN_ID" == 198 && "$ROS_LOCALHOST_ONLY" == 1 ]]
run="$here/${TURN_SIM_RUN_ID:?}"
mkdir "$run"
exec > >(tee "$run/run.log") 2>&1
id=shaki-turn-memory-sim
for component in nav map scan sim fixture; do
  if systemctl --user is-active --quiet "$id-$component"; then exit 2; fi
done
cleanup() { systemctl --user stop "$id-nav" "$id-map" "$id-scan" "$id-sim" "$id-fixture" >/dev/null 2>&1 || true; }
trap cleanup EXIT
existing=$(timeout 8 ros2 node list 2>/dev/null || true)
[[ -z "$existing" ]] || { echo "Simulation domain occupied: $existing"; exit 2; }
cp "$here/receipt.json" "$run/"
systemd-run --user --unit="$id-sim" --collect --property=TimeoutStopSec=10 \
  --setenv=SIM_RMW=rmw_cyclonedds_cpp --setenv=SIM_SCENE=straight_wall \
  /usr/bin/bash "$root/start_isolated_sim.sh"
systemd-run --user --unit="$id-scan" --collect --property=TimeoutStopSec=10 --setenv=SIM_RMW=rmw_cyclonedds_cpp \
  /usr/bin/bash "$root/start_isolated_scan.sh"
systemd-run --user --unit="$id-map" --collect --property=TimeoutStopSec=10 --setenv=SIM_RMW=rmw_cyclonedds_cpp \
  /usr/bin/bash "$root/start_isolated_map.sh"
sleep 10
profile=${TURN_SIM_PROFILE:-simulation.yaml}
if [[ ${TURN_UNKNOWN_TEST:-0} == 1 ]]; then
  profile=simulation_unknown.yaml
  systemd-run --user --unit="$id-fixture" --collect --setenv=SIM_RMW=rmw_cyclonedds_cpp \
    /usr/bin/bash "$here/start_unknown_fixture.sh"
fi
systemd-run --user --unit="$id-nav" --collect --property=TimeoutStopSec=10 --setenv=SIM_RMW=rmw_cyclonedds_cpp \
  --setenv="TURN_PROFILE=$profile" \
  /usr/bin/bash "$here/start_sim_nav.sh"
ready=false
for attempt in $(seq 1 45); do
  systemctl --user is-active --quiet "$id-nav" || break
  if timeout 4 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active'; then ready=true; break; fi
  sleep 1
done
if [[ "$ready" == true ]]; then
  if [[ ${TURN_UNKNOWN_TEST:-0} == 1 ]]; then
    python3 "$here/assert_unknown_policy.py" "$run/unknown_policy.json"
  fi
  set +e
  if [[ ${TURN_NATIVE_SPIN_TEST:-0} == 1 ]]; then
    python3 "$root/lattice_arc_20261031/test_native_spin.py" "$run/native_spin.json" \
      --ros-args -p use_sim_time:=true > "$run/test.log" 2>&1
  else
    python3 "$here/test_navigation.py" "$run/telemetry.json" \
      --ros-args -p use_sim_time:=true > "$run/test.log" 2>&1
  fi
  result=$?
  set -e
else
  result=125
fi
echo "$result" > "$run/exit_status"
journalctl --user -u "$id-nav" --no-pager -o cat > "$run/nav.log" || true
cleanup
echo complete > "$run/COMPLETE"
exit "$result"
