#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/whill/Shaki_ws/dds_eval_20261030
mkdir "$ROOT"
mkdir "$ROOT/packages" "$ROOT/stage"
cd "$ROOT/packages"
apt-get download ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds \
  ros-humble-iceoryx-binding-c ros-humble-iceoryx-hoofs ros-humble-iceoryx-posh
for package in ./*.deb; do dpkg-deb -x "$package" "$ROOT/stage"; done
sha256sum ./*.deb
LD_LIBRARY_PATH="$ROOT/stage/opt/ros/humble/lib:$ROOT/stage/opt/ros/humble/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}" \
  ldd "$ROOT/stage/opt/ros/humble/lib/librmw_cyclonedds_cpp.so"
