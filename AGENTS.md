# Repository Guidelines

## Project Structure & Module Organization

`rl_sar` is a C++ robotics deployment framework for reinforcement-learning locomotion policies. Main controller code lives in `src/rl_sar/`: entry points are under `src/rl_sar/src/`, shared SDK and inference code under `src/rl_sar/library/core/`, robot FSMs under `src/rl_sar/fsm_robot/`, and launch/world assets under `src/rl_sar/launch/` and `src/rl_sar/worlds/`. Robot descriptions live in `src/rl_sar_zoo/<robot>_description/`. Policy assets live in `policy/<robot>/<config>/` with shared robot settings in `policy/<robot>/base.yaml`. Helper scripts are in `scripts/`; Docker setup is in `docker/`. Build outputs such as `build/`, `install/`, `devel/`, `log/`, and `cmake_build/` are generated.

## Build, Test, and Development Commands

Use `./build.sh` as the normal entry point.

- `./build.sh`: build all ROS packages using the sourced `$ROS_DISTRO`.
- `./build.sh rl_sar`: build only the main package.
- `./build.sh -m` or `./build.sh --cmake`: CMake-only build for non-ROS deployment.
- `./build.sh -mj` or `./build.sh --mujoco`: CMake build with MuJoCo simulator support.
- `./build.sh -c`: clean generated build artifacts and package symlinks.

After ROS builds, source `devel/setup.bash` for ROS1 or `install/setup.bash` for ROS2. Example run commands: `rosrun rl_sar rl_sim`, `ros2 run rl_sar rl_sim`, or `./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>`.

## Coding Style & Naming Conventions

The project uses C++17. Follow existing C++ style in nearby files: 4-space indentation, descriptive class names such as `RL_Sim_Mujoco`, and robot-specific files named `fsm_<robot>.hpp` or `rl_real_<robot>.cpp`. Keep YAML keys consistent with existing policy configs. Pre-commit inserts Apache-2.0 license headers for `.py`, `.yaml`, `.cpp`, and `.hpp` files; run `pre-commit run --all-files` when available.

## Testing Guidelines

There is no fully wired test command documented. Test sources exist in `src/rl_sar/test/`, but related CMake targets are currently commented out. For changes, at minimum run the relevant `./build.sh` mode and, when behavior changes, validate in Gazebo, MuJoCo, or the target hardware path. Be especially careful with `joint_names` and `joint_mapping`; incorrect order can create unsafe behavior.

## Commit & Pull Request Guidelines

Recent history uses short conventional-style commits, for example `feat(takeoff): ...` and `chore: ...`. Prefer concise subjects with a scope when useful. Pull requests should describe the target robot/runtime, build mode tested, policy/config files touched, and any simulator or hardware validation performed. Link related issues and include logs or screenshots only when they clarify failures or runtime behavior.

## Agent-Specific Instructions

Do not invoke `catkin`, `colcon`, or `cmake` directly unless debugging `build.sh`. Avoid editing downloaded vendor/runtime directories under `library/inference_runtime/` or generated robot descriptions unless the task explicitly requires it.
