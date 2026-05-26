#!/usr/bin/env bash
set -euo pipefail

# C3 sonar + DAVE helper setup script
# NOTE: DAVE multibeam plugin is ROS1/Catkin oriented. We keep it isolated from ROS2 workspace.

DAVE_WS="${DAVE_WS:-/data/code/dave_ws}"
SRC_DIR="$DAVE_WS/src"
DAVE_REPO="https://github.com/Field-Robotics-Lab/dave.git"
MULTIBEAM_REPO="https://github.com/Field-Robotics-Lab/nps_uw_multibeam_sonar.git"

mkdir -p "$SRC_DIR"

clone_retry() {
  local url="$1"
  local dst="$2"
  local name="$3"
  rm -rf "$dst"
  for i in 1 2 3 4 5; do
    if git clone --depth=1 --single-branch "$url" "$dst"; then
      return 0
    fi
    rm -rf "$dst"
    sleep $((i*3))
  done
  echo "[$name] clone failed after retries" >&2
  return 1
}

if [ ! -d "$SRC_DIR/dave/.git" ]; then
  clone_retry "$DAVE_REPO" "$SRC_DIR/dave" "dave"
fi

if [ ! -d "$SRC_DIR/nps_uw_multibeam_sonar/.git" ]; then
  clone_retry "$MULTIBEAM_REPO" "$SRC_DIR/nps_uw_multibeam_sonar" "nps_uw_multibeam_sonar"
fi
