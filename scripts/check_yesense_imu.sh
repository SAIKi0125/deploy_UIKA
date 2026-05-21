#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${UIKA_HARDWARE_WS:-}" ]]; then
  HARDWARE_WS="${UIKA_HARDWARE_WS}"
elif [[ -d /home/nvidia/Uika/Quadruped_Uika ]]; then
  HARDWARE_WS="/home/nvidia/Uika/Quadruped_Uika"
else
  HARDWARE_WS="/home/saiki/project/Uika/Quadruped_Uika"
fi

source /opt/ros/humble/setup.bash
source "${HARDWARE_WS}/install/setup.bash"

export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-1}"

STARTED_PID=""
cleanup() {
  if [[ -n "${STARTED_PID}" ]] && kill -0 "${STARTED_PID}" 2>/dev/null; then
    kill -INT "${STARTED_PID}" 2>/dev/null || true
    wait "${STARTED_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if ! ros2 topic list | grep -qx "/imu/data"; then
  echo "[imu] /imu/data is not active; starting yesense_std_ros2"
  ros2 launch yesense_std_ros2 yesense_node.launch.py &
  STARTED_PID=$!
  sleep 2
else
  echo "[imu] using existing /imu/data publisher"
fi

echo "[imu] topic info"
ros2 topic info /imu/data -v || true

echo "[imu] one /imu/data frame"
timeout 8 ros2 topic echo /imu/data --once

echo "[imu] one /imu/rpy frame"
timeout 4 ros2 topic echo /imu/rpy --once || true
