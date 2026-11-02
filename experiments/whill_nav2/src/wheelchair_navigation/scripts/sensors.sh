#!/usr/bin/env bash
set -eo pipefail
ws=/home/whill/Shaki_ws/src/WheelChair_ws
pkg="$ws/src/wheelchair_navigation"
unit=shaki-nav2-sensors-20261101.service
action=${1:-start}
case "$action" in
  status) exec systemctl --user status "$unit" --no-pager ;;
  stop) exec systemctl --user stop "$unit" ;;
  restart) systemctl --user stop "$unit" ;;
  start) ;;
  *) echo 'Usage: sensors.sh [start|restart|stop|status]' >&2; exit 2 ;;
esac
if ! systemctl --user is-active --quiet "$unit"; then
  systemctl --user reset-failed "$unit" 2>/dev/null || true
  systemd-run --user --unit="$unit" --collect /usr/bin/bash "$pkg/support/first_run/start_sensors.sh"
fi
source "$pkg/profiles/turn_memory/env.sh"
source "$pkg/support/first_run/dds_profile.env"
python3 "$pkg/support/first_run/check_sensor_ready.py"
echo 'Dual MID360 and Odin are fresh. Sensor-only startup does not open RViz.'
