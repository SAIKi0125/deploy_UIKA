# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

`rl_sar` ("simulation and real") is a C++ framework for deploying reinforcement-learning locomotion policies on quadruped, wheeled, and humanoid robots. The same controller code runs in Gazebo, MuJoCo, and on physical hardware; the target is selected at build/runtime, not by editing the policy.

Supported environments are dual-stack: **ROS1 (noetic) ↔ ROS2 (foxy/humble)**, **libtorch ↔ onnxruntime**, **Gazebo ↔ MuJoCo**, **Linux ↔ macOS (MuJoCo only)**. Python training-side code is suspended — use [v2.3](https://github.com/fan-ziqi/rl_sar/releases/tag/v2.3) if needed.

## Build Commands

Everything goes through `./build.sh`; do not call `catkin`/`colcon`/`cmake` directly unless debugging.

```bash
./build.sh                    # ROS build (all packages). Requires $ROS_DISTRO sourced.
./build.sh rl_sar             # ROS build, specific package(s)
./build.sh -m  | --cmake      # CMake-only, no ROS. Outputs to cmake_build/{bin,lib}. For hardware deployment.
./build.sh -mj | --mujoco     # CMake + MuJoCo simulator support
./build.sh -c  | --clean      # Remove symlinks + build/ cmake_build/ devel/ install/ log/ .catkin_tools/
./build.sh -h                 # Full usage
```

Key behaviors of `build.sh` that matter when debugging build problems:
- It auto-runs the `scripts/download_*.sh` helpers to fetch the inference runtime, MuJoCo, robot descriptions, and Gazebo models on first build.
- It symlinks `package.ros1.xml` or `package.ros2.xml` → `package.xml` per package based on `$ROS_DISTRO`. Switching between ROS1 and ROS2 in the same checkout will auto-clean incompatible artifacts. If a package fails to be recognized, check that the symlink points at the right variant.
- ROS1 uses `catkin build`; ROS2 uses `colcon build --merge-install --symlink-install`.
- ROS1 `empy` error → run `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3` before building.
- Jetson platforms auto-disable ONNX Runtime (detected via `/etc/nv_tegra_release`).

## Running

Source the overlay first: `source devel/setup.bash` (ROS1) or `source install/setup.bash` (ROS2).

| Target | ROS1 | ROS2 | CMake (no ROS) |
|---|---|---|---|
| Gazebo sim | `roslaunch rl_sar gazebo.launch rname:=<ROBOT>` + `rosrun rl_sar rl_sim` | `ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>` + `ros2 run rl_sar rl_sim` | — |
| MuJoCo sim | — | — | `./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>` |
| Real robot | `rosrun rl_sar rl_real_<ROBOT> [args]` | `ros2 run rl_sar rl_real_<ROBOT> [args]` | `./cmake_build/bin/rl_real_<ROBOT> [args]` |

For Gazebo you **must** launch `rl_sim` in a second terminal or the robot will fall. Real-robot entry points are `rl_real_a1`, `rl_real_go2` (takes `<NETWORK_INTERFACE> [wheel]`), `rl_real_g1` (network interface), `rl_real_lite3`, `rl_real_d1` (`[local_ip] [robot_ip]`), `rl_real_l4w4`.

Docker: see `docker/README.md`. `cd docker && docker compose up -d` builds a ROS2 Humble + Gazebo + MuJoCo image; policies are bind-mounted read-only from `../policy`.

There is no test suite or lint command wired up. Pre-commit (`.pre-commit-config.yaml`) only inserts Apache-2.0 license headers into `.py/.yaml/.cpp/.hpp`.

## Architecture

### Top-level layout

- `src/rl_sar/` — main ROS package. Controller entry points (`rl_sim.cpp`, `rl_sim_mujoco.cpp`, `rl_real_*.cpp`), Gazebo launch files, per-robot FSMs, and the `library/core` that everything links against.
- `src/rl_sar_zoo/<ROBOT>_description/` — URDF/xacro + Gazebo config per robot. Auto-downloaded by `scripts/download_robot_descriptions.sh`; not normally edited in-tree.
- `src/robot_joint_controller/` and `src/robot_msgs/` — custom ros_control controller and message types used on the Gazebo side.
- `policy/<ROBOT>/<CONFIG>/` — trained models (`.pt` or `.onnx`) + `config.yaml` + per-robot `base.yaml`. Selected at runtime; no recompile needed to swap policies.
- `library/inference_runtime/` — downloaded libtorch and onnxruntime binaries. Populated by `scripts/download_inference_runtime.sh`.
- `cmake_build/` (non-ROS builds) vs `build/`+`devel/`+`install/` (ROS builds). These two layouts are mutually incompatible — `build.sh` cleans the other when you switch.

### The `RL` base class (`src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`)

Every entry point (`RL_Sim`, `RL_Sim_Mujoco`, `RL_Real_*`) inherits from `RL`. Subclasses implement three pure virtuals:

- `GetState(RobotState<float>*)` — pull IMU + joint state from Gazebo/MuJoCo/SDK.
- `SetCommand(const RobotCommand<float>*)` — push `q/dq/kp/kd/tau` back out.
- `Forward()` — assemble observation vector, run the inference model, return actions.

`StateController` orchestrates the loop; `ComputeObservation`, `ComputeOutput`, `history_obs_buf`, `TorqueProtect`, and `AttitudeProtect` are shared. Inference is abstracted by `InferenceRuntime::Model` (libtorch + ONNX implementations behind a common interface); the backend is picked from the file extension in `config.yaml`.

### Config resolution

Two YAML files are read at startup:
- `policy/<ROBOT>/base.yaml` — physical-robot properties (joint order, default pose, torque limits). **`joint_names` must match the real robot's joint order; `joint_mapping` is what bridges training-frame order to that order.**
- `policy/<ROBOT>/<CONFIG>/config.yaml` — policy-specific (model file, observation terms, scales, history window, rl_kp/rl_kd).

`rl_sim` picks `<CONFIG>` at runtime; `<ROBOT>` comes from a ROS param (`rname:=`) in ROS builds or from argv in CMake/MuJoCo builds. `YamlParams::Get<T>(key)` is the only accessor — store containers in a local before calling `.begin()`; the header comments about dangling iterator references are load-bearing.

### Finite State Machine

Each robot has its own FSM in `src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp` (Passive, GetUp, GetDown, RL, etc.). Registration uses the `REGISTER_FSM_FACTORY` macro in `library/core/fsm/fsm.hpp`, and controllers look up the factory by `robot_name` through `FSMManager::GetInstance()`. `fsm_all.hpp` just includes every per-robot header so the static registrations all fire. When adding a robot, include its new header in `fsm_all.hpp` too.

### Input

Keyboard (`Input::Keyboard`) and gamepad (`Input::Gamepad`) enums in `rl_sdk.hpp` are the single source of truth for control bindings; FSM transitions read `rl.control.current_keyboard`/`current_gamepad`. See README for the full button map — key ones: `Num0`/`A` stand up, `Num9`/`B` lie down, `Num1`/`RB+DPadUp` basic locomotion, `R`/`RB+Y` reset Gazebo.

## Adding a Robot

Names must match exactly. Minimum set (see `go2w` as reference):

```
src/rl_sar_zoo/<ROBOT>_description/   # xacro + gazebo config + package.ros{1,2}.xml + CMakeLists.txt
policy/<ROBOT>/base.yaml              # joint order must match physical robot
policy/<ROBOT>/<CONFIG>/config.yaml
policy/<ROBOT>/<CONFIG>/<POLICY>.pt   # JIT-exported torchscript, OR
policy/<ROBOT>/<CONFIG>/<POLICY>.onnx
src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp  # then add an #include in fsm_all.hpp
src/rl_sar/src/rl_real_<ROBOT>.cpp    # override Forward() if obs layout differs
```

`scripts/convert_policy.py` auto-detects and converts between `.pt` and `.onnx`. `scripts/actuator_net.py` trains an actuator net from CSV data collected by enabling `#define CSV_LOGGER` in an `rl_real_*.hpp`.

## Gotchas

- Joint order is the #1 source of silent-but-unsafe bugs. Training-frame order ≠ real-robot order in most configs; `joint_mapping` in `config.yaml` is what reconciles them. Double-check when porting a policy.
- ROS1 ↔ ROS2 switches require `./build.sh -c` if the auto-cleanup doesn't catch it (it compares artifact directories, not source state).
- `ang_vel_axis` is `"world"` in ROS1 `rl_sim` and `"body"` in ROS2 `rl_sim` — observation math depends on this, don't cross the streams.
- `rl_sar_zoo` and the inference runtime are downloaded lazily by `build.sh`; `.gitmodules` only lists the vendor SDK submodules (unitree, deeprobotics, agibot, joystick).
