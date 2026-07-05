#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/nvidia/Uika/Quadruped_Uika/install/setup.bash

export CUDACXX=/usr/local/cuda-12.6/bin/nvcc
export CUDA_HOME=/usr/local/cuda-12.6
export PATH=/usr/local/cuda-12.6/bin:${PATH}
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH:-}

cd "$(dirname "$0")/.."
./build.sh robot_msgs rl_sar
test -x install/lib/rl_sar/rl_real_uika
echo "RL_REAL_UIKA_OK"
