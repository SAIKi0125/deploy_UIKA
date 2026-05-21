#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${UIKA_ROOT:-}" ]]; then
  UIKA_ROOT_DIR="${UIKA_ROOT}"
elif [[ -d /home/nvidia/Uika ]]; then
  UIKA_ROOT_DIR="/home/nvidia/Uika"
else
  UIKA_ROOT_DIR="/home/saiki/project/Uika"
fi

HARDWARE_WS="${UIKA_HARDWARE_WS:-${UIKA_ROOT_DIR}/Quadruped_Uika}"

if command -v conda >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "$(conda info --base)/etc/profile.d/conda.sh" || true
  conda deactivate >/dev/null 2>&1 || true
fi

if [[ -x "${UIKA_ROOT_DIR}/start_uika.sh" ]]; then
  echo "Starting UIKA hardware via ${UIKA_ROOT_DIR}/start_uika.sh"
  echo "That bringup defaults to imu_type:=yesense."
  exec "${UIKA_ROOT_DIR}/start_uika.sh"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Missing /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${HARDWARE_WS}/install/setup.bash" ]]; then
  echo "Missing hardware workspace setup: ${HARDWARE_WS}/install/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${HARDWARE_WS}/install/setup.bash"
set -u

echo "Starting bringup with imu_type:=yesense"
exec ros2 launch bringup bringup.launch.py imu_type:=yesense
