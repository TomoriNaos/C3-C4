#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# Prevent stale deleted install paths from previous overlays from confusing colcon.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PYTHONPATH

set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install "$@"
