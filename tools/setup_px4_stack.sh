#!/usr/bin/env bash
set -euo pipefail

# 仅准备本项目所需 PX4 相关依赖：
# 1) PX4-Autopilot
# 2) px4_msgs (ROS2)
# 3) Micro-XRCE-DDS-Agent

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_DIR="${ROOT_DIR}/third_party"
PX4_DIR="${THIRD_PARTY_DIR}/PX4-Autopilot"
PX4_ROS2_WS="${THIRD_PARTY_DIR}/px4_ros2_ws"
PX4_MSGS_DIR="${PX4_ROS2_WS}/src/px4_msgs"
XRCE_DIR="${THIRD_PARTY_DIR}/Micro-XRCE-DDS-Agent"

mkdir -p "${THIRD_PARTY_DIR}"

echo "[1/4] Clone PX4-Autopilot (if needed)"
if [[ ! -d "${PX4_DIR}" ]]; then
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive "${PX4_DIR}"
else
  echo "  PX4-Autopilot already exists: ${PX4_DIR}"
fi

echo "[2/4] Install PX4 Ubuntu dependencies"
cd "${PX4_DIR}"
bash ./Tools/setup/ubuntu.sh

echo "[3/4] Build px4_msgs in local ROS2 workspace"
mkdir -p "${PX4_ROS2_WS}/src"
if [[ ! -d "${PX4_MSGS_DIR}" ]]; then
  git clone https://github.com/PX4/px4_msgs.git "${PX4_MSGS_DIR}"
else
  echo "  px4_msgs already exists: ${PX4_MSGS_DIR}"
fi

cd "${PX4_ROS2_WS}"
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs

echo "[4/4] Build Micro-XRCE-DDS-Agent"
if [[ ! -d "${XRCE_DIR}" ]]; then
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${XRCE_DIR}"
else
  echo "  Micro-XRCE-DDS-Agent already exists: ${XRCE_DIR}"
fi

cd "${XRCE_DIR}"
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"

echo "Install XRCE agent (sudo required):"
echo "  sudo make install && sudo ldconfig"

echo
echo "Done. Next steps:"
echo "  1) source /opt/ros/humble/setup.bash"
echo "  2) source ${PX4_ROS2_WS}/install/setup.bash"
echo "  3) MicroXRCEAgent udp4 -p 8888"
