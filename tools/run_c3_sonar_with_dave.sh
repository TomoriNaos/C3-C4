#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${ROOT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DAVE_WS="${DAVE_WS:-/data/code/dave_ws}"
AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}:$DAVE_WS/devel/lib:/opt/ros/noetic/lib"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$DAVE_WS/src/dave/models:$DAVE_WS/src/nps_uw_multibeam_sonar/models"

ros2 launch c3_sonar_driver sonar_core.launch.py \
  enable_dave_plugin:=true \
  enable_dave_adapter:=true \
  dave_pointcloud_topic:=/multibeam_sonar_point_cloud
