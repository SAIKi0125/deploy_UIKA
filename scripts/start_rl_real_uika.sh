#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${UIKA_HARDWARE_WS:-}" ]]; then
  HARDWARE_WS="${UIKA_HARDWARE_WS}"
elif [[ -d /home/nvidia/Uika/Quadruped_Uika ]]; then
  HARDWARE_WS="/home/nvidia/Uika/Quadruped_Uika"
else
  HARDWARE_WS="/home/saiki/project/Uika/Quadruped_Uika"
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROOT_DIR}/logs/ros}"
mkdir -p "${ROS_LOG_DIR}"

if command -v conda >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "$(conda info --base)/etc/profile.d/conda.sh" || true
  conda deactivate >/dev/null 2>&1 || true
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Missing /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${HARDWARE_WS}/install/setup.bash" ]]; then
  echo "Missing hardware workspace setup: ${HARDWARE_WS}/install/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "Missing deploy_UIKA install setup: ${ROOT_DIR}/install/setup.bash" >&2
  echo "Build first with ROS2 sourced: ./build.sh robot_msgs robot_joint_controller rl_sar" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${HARDWARE_WS}/install/setup.bash"
# shellcheck disable=SC1091
source "${ROOT_DIR}/install/setup.bash"
set -u

echo "Using hardware workspace: ${HARDWARE_WS}"
echo "Expecting IMU on /imu/data from yesense_std_ros2."
exec ros2 run rl_sar rl_real_uika "$@"
