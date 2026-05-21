#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
HARDWARE_SETUP="${UIKA_HARDWARE_SETUP:-/home/nvidia/Uika/Quadruped_Uika/install/setup.bash}"
DEPLOY_SETUP="${UIKA_DEPLOY_SETUP:-${ROOT_DIR}/install/setup.bash}"

source "${ROS_SETUP}"
source "${HARDWARE_SETUP}"
source "${DEPLOY_SETUP}"

export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-1}"

cleanup() {
    if [[ -n "${RL_PID:-}" ]] && kill -0 "${RL_PID}" 2>/dev/null; then
        kill -INT "${RL_PID}" 2>/dev/null || true
        for _ in {1..20}; do
            if ! kill -0 "${RL_PID}" 2>/dev/null; then
                wait "${RL_PID}" 2>/dev/null || true
                return
            fi
            sleep 0.1
        done
        kill -TERM "${RL_PID}" 2>/dev/null || true
        for _ in {1..20}; do
            if ! kill -0 "${RL_PID}" 2>/dev/null; then
                wait "${RL_PID}" 2>/dev/null || true
                return
            fi
            sleep 0.1
        done
        kill -KILL "${RL_PID}" 2>/dev/null || true
        wait "${RL_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "[verify] starting rl_real_uika"
RL_BIN="${ROOT_DIR}/install/lib/rl_sar/rl_real_uika"
if [[ ! -x "${RL_BIN}" ]]; then
    echo "[verify] missing executable: ${RL_BIN}" >&2
    exit 1
fi
"${RL_BIN}" "$@" &
RL_PID=$!

sleep 2
if ! kill -0 "${RL_PID}" 2>/dev/null; then
    echo "[verify] rl_real_uika exited early" >&2
    wait "${RL_PID}"
fi

echo "[verify] running fake golden frame"
python3 "${ROOT_DIR}/scripts/fake_uika_golden_frame.py" --duration 10 --min-commands 5
cleanup
trap - EXIT
echo "[verify] PASS"
