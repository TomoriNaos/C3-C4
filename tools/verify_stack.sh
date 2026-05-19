#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

MODE="${1:-sim}"  # sim | px4
if [[ "$MODE" != "sim" && "$MODE" != "px4" ]]; then
  echo "Usage: $0 [sim|px4]"
  exit 1
fi

echo "[1/5] Build c3_drone_driver ..."
colcon build --packages-select c3_drone_driver --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[2/5] Check launch syntax ..."
python3 -m py_compile \
  c3_drone_driver/launch/c3_drone_core.launch.py \
  c3_drone_driver/launch/c3_gazebo_sim.launch.py

echo "[3/5] Check required binaries ..."
BIN_DIR="build/c3_drone_driver"
required_bins=(
  motion_controller_node
  drone_main_controller_node
  gimbal_controller_node
  mavlink_bridge_node
  target_processor_node
  px4_pose_bridge_node
  offboard_setpoint_bridge_node
)
for b in "${required_bins[@]}"; do
  if [[ ! -x "${BIN_DIR}/${b}" ]]; then
    echo "ERROR: missing binary ${BIN_DIR}/${b}"
    exit 1
  fi
done
echo "Binaries OK."

if [[ "$MODE" == "px4" ]]; then
  if [[ -x "${BIN_DIR}/offboard_setpoint_px4_bridge_node" ]]; then
    echo "PX4 native bridge binary detected: offboard_setpoint_px4_bridge_node"
  else
    echo "WARNING: offboard_setpoint_px4_bridge_node not built."
    echo "         This usually means px4_msgs is not installed/sourced."
  fi
fi

echo "[4/5] Runtime ROS graph checks (requires your stack is already running) ..."
if ! command -v ros2 >/dev/null 2>&1; then
  echo "WARNING: ros2 not found in PATH, skip runtime checks."
  exit 0
fi

set +e
ros2 node list > /tmp/c3_nodes.txt 2>/dev/null
rc_nodes=$?
ros2 topic list > /tmp/c3_topics.txt 2>/dev/null
rc_topics=$?
set -e

if [[ $rc_nodes -ne 0 || $rc_topics -ne 0 ]]; then
  echo "WARNING: ROS graph unavailable. Source ROS env and launch your stack first."
  echo "Hint:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  source install/setup.bash"
  exit 0
fi

echo "[5/5] Validate key topics ..."
required_topics_common=(
  /px4/vehicle_pose
  /px4/offboard_goal
  /mission/goal
  /mission/cmd
  /gimbal/state
  /main_controller/status
  /ship/pose_world
  /ship/target_point
)

required_topics_px4=(
  /fmu/in/offboard_control_mode
  /fmu/in/trajectory_setpoint
  /fmu/in/vehicle_command
  /fmu/out/vehicle_odometry
)

required_topics=("${required_topics_common[@]}")
if [[ "$MODE" == "px4" ]]; then
  required_topics+=("${required_topics_px4[@]}")
fi

missing=0
for t in "${required_topics[@]}"; do
  if ! grep -qx "$t" /tmp/c3_topics.txt; then
    echo "MISSING topic: $t"
    missing=1
  fi
done

if [[ $missing -eq 1 ]]; then
  echo "Runtime check finished with missing topics (mode=${MODE})."
  exit 2
fi

echo "All checks passed (mode=${MODE})."
