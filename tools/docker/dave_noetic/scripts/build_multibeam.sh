#!/usr/bin/env bash
set -euo pipefail

DAVE_WS="${DAVE_WS:-/workspaces/dave_ws}"
SRC_DIR="$DAVE_WS/src"
RUN_ROSDEP="${RUN_ROSDEP:-0}"
AUTO_INSTALL_CUDA="${AUTO_INSTALL_CUDA:-0}"

source /opt/ros/noetic/setup.bash

if [ ! -d "$SRC_DIR/dave" ] || [ ! -d "$SRC_DIR/nps_uw_multibeam_sonar" ]; then
  echo "[build_multibeam] Missing source repositories under $SRC_DIR" >&2
  echo "Expected: dave and nps_uw_multibeam_sonar" >&2
  exit 1
fi

mkdir -p "$DAVE_WS"
cd "$DAVE_WS"

if [ ! -f "$DAVE_WS/src/CMakeLists.txt" ]; then
  catkin_init_workspace "$DAVE_WS/src"
fi

if [ "$RUN_ROSDEP" = "1" ]; then
  rosdep update || true
  rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys="uuv_sensor_ros_plugins marine_acoustic_msgs" || true
fi

if ! command -v nvcc >/dev/null 2>&1; then
  if [ "$AUTO_INSTALL_CUDA" = "1" ]; then
    apt-get update
    apt-get install -y --no-install-recommends nvidia-cuda-toolkit
  fi
fi

if ! command -v nvcc >/dev/null 2>&1; then
  echo "nvcc not found. Install CUDA toolkit in container:" >&2
  echo "  apt-get update && apt-get install -y --no-install-recommends nvidia-cuda-toolkit" >&2
  echo "Or rerun with AUTO_INSTALL_CUDA=1 build_multibeam.sh" >&2
  exit 1
fi

# Build only what we need for multibeam sonar plugins first.
catkin_make \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES="nps_uw_multibeam_sonar;uuv_sensor_ros_plugins;marine_acoustic_msgs"
