#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

OUTPUT_DIR="${1:-/home/hu/usv_captures/annotation_raw}"
MAX_IMAGES="${2:-96}"

rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

export GAZEBO_IP=127.0.0.1
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch usv_bringup annotation_capture.launch.py \
  output_dir:="$OUTPUT_DIR" \
  max_images:="$MAX_IMAGES" \
  every_n:=1 \
  gui:=false
