#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

export GAZEBO_IP=127.0.0.1
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch usv_bringup sim.launch.py "$@"
