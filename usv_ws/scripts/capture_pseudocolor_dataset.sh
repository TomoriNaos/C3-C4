#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

SINGLE_DIR="${1:-/home/hu/usv_captures/pseudocolor_single}"
COMPLEX_DIR="${2:-/home/hu/usv_captures/pseudocolor_complex}"
SINGLE_COUNT="${3:-96}"
COMPLEX_COUNT="${4:-96}"

export GAZEBO_IP=127.0.0.1
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

rm -rf "$SINGLE_DIR" "$COMPLEX_DIR"
mkdir -p "$SINGLE_DIR" "$COMPLEX_DIR"

ros2 launch usv_bringup annotation_pseudocolor_capture.launch.py \
  output_dir:="$SINGLE_DIR" \
  max_images:="$SINGLE_COUNT" \
  every_n:=1 \
  occlusion_mode:=false \
  gui:=false

ros2 launch usv_bringup annotation_pseudocolor_capture.launch.py \
  output_dir:="$COMPLEX_DIR" \
  max_images:="$COMPLEX_COUNT" \
  every_n:=5 \
  occlusion_mode:=true \
  gui:=false
