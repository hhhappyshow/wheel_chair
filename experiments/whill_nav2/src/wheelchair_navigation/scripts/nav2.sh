#!/usr/bin/env bash
set -eo pipefail
ws=/home/whill/Shaki_ws/src/WheelChair_ws
pkg="$ws/src/wheelchair_navigation"
profile=${1:-turn_memory}
action=${2:-start}
case "$profile" in
  turn_memory) entry="$pkg/profiles/turn_memory/start_navigation.sh" ;;
  *) echo 'Profile: turn_memory' >&2; exit 2 ;;
esac
unit=shaki-nav2-demo.service
case "$action" in
  check) exec bash "$entry" --check ;;
  status) exec systemctl --user status "$unit" --no-pager ;;
  stop) exec systemctl --user stop "$unit" ;;
  start|restart) ;;
  *) echo 'Actions: start | restart | stop | status | check' >&2; exit 2 ;;
esac
if systemctl --user is-active --quiet "$unit" && [[ "$action" == start ]]; then
  active=$(cat "$ws/records/active_profile.txt" 2>/dev/null || true)
  if [[ "$active" == "$profile" ]]; then
    echo "$profile already running. Use: bash $ws/scripts/nav2.sh $profile restart"
    exit 0
  fi
  echo "Currently running $active; use '$profile restart' to switch." >&2
  exit 1
fi
bash "$entry" --check
source "$pkg/profiles/turn_memory/env.sh"
source "$pkg/support/first_run/dds_profile.env"
python3 "$pkg/support/first_run/check_sensor_ready.py"
# All targets are this task's owned navigation units; leave sensor service running.
for owned in "$unit" shaki-nav2-turn-memory-20261101.service shaki-nav2-lattice10-live-20261101.service; do
  if systemctl --user is-active --quiet "$owned"; then systemctl --user stop "$owned"; fi
done
systemctl --user reset-failed "$unit" 2>/dev/null || true
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
systemd-run --user --unit="$unit" --collect \
  --setenv="DISPLAY=$DISPLAY" --setenv="XAUTHORITY=$XAUTHORITY" \
  /usr/bin/bash "$entry"
printf '%s\n' "$profile" > "$ws/records/active_profile.txt"
python3 "$pkg/support/first_run/check_nav2_ready.py"
echo "Nav2 profile=$profile launched. ARM in the controller window; RViz 2D Goal Pose starts navigation."
