# deploy_UIKA

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)
[![ROS2 Foxy/Humble](https://img.shields.io/badge/ros2-foxy/humble-brightgreen.svg?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-lightgrey.svg?logo=gazebo)](http://gazebosim.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2.7-orange.svg?logo=mujoco)](https://mujoco.org/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

UIKA 四足机器人的 Sim2Real 部署仓库,基于 [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 框架裁剪而来,只保留 UIKA 一台机器人。配套的训练仓库是 [himloco_lab](https://github.com/SAIKi0125/UIKA_lab) ── 在 Isaac Lab 中训练 → 导出 `policy.pt` / `deploy.yaml` → 拷进本仓库 → Gazebo / MuJoCo 验证 → 实机部署。

> [!CAUTION]
> **免责声明:使用本代码所产生的一切风险与后果由用户自行承担,作者不对任何直接或间接损失负责。实机部署前请务必准备好急停、限位、保护绳。**

## 目录

- [仓库定位](#仓库定位)
- [仓库结构](#仓库结构)
- [环境准备](#环境准备)
- [编译](#编译)
- [运行](#运行)
- [策略与训练仓库的对应关系](#策略与训练仓库的对应关系)
- [部署完整流程](#部署完整流程)
- [FSM 状态与按键](#fsm-状态与按键)
- [参考项目](#参考项目)

## 仓库定位

本仓库 = `rl_sar` 框架 + UIKA 专属资源。相比上游 `rl_sar`:

- 只保留 UIKA 一台机器人(`policy/uika/`、`fsm_uika.hpp`、`uika_description/`)
- 去掉了 a1/go2/go2w/g1/b2/b2w/d1/lite3/l4w4/gr1t1/gr1t2/tita 等非 UIKA 资源与对应可执行文件
- 通用入口 `rl_sim`(Gazebo)、`rl_sim_mujoco`(MuJoCo)保留,实机入口尚未就绪 ── 实机控制可走 ROS 通用入口或自行新增 `rl_real_uika.cpp`
- `src/rl_sar_zoo/uika_description/` 已直接随仓库分发,无需另外执行 `download_robot_descriptions.sh`

UIKA 训练侧的策略结构、奖励设计、地形配置等请参阅 [himloco_lab](https://github.com/SAIKi0125/UIKA_lab) ── 本仓库只关心训练好的策略如何跑起来。

## 仓库结构

```text
deploy_UIKA/
├── build.sh                              # 一键构建脚本(详见“编译”一节)
├── policy/uika/
│   ├── base.yaml                         # UIKA 物理参数:关节顺序、默认站立角、扭矩上限
│   └── himloco/
│       ├── config.yaml                   # 策略参数:观测构成、归一化、动作缩放、命令限幅
│       └── policy.pt                     # 来自 himloco_lab 的 TorchScript 策略
├── src/rl_sar/
│   ├── library/core/                     # rl_sdk、推理后端、FSM、observation buffer 等通用核心
│   ├── fsm_robot/
│   │   ├── fsm_uika.hpp                  # UIKA 状态机:Passive / GetUp / GetDown / RLHimLoco
│   │   └── fsm_all.hpp                   # 仅 include fsm_uika.hpp
│   ├── src/
│   │   ├── rl_sim.cpp                    # Gazebo 入口
│   │   └── rl_sim_mujoco.cpp             # MuJoCo 入口
│   ├── launch/                           # gazebo.launch / gazebo.launch.py
│   └── test/test_uika_integration.py     # 校验 deploy 配置与训练导出 deploy.yaml 一致
├── src/rl_sar_zoo/uika_description/      # UIKA URDF / Mesh / MJCF
│   ├── urdf/uika_description.urdf
│   ├── mjcf/{uika.xml, scene.xml}
│   └── meshes/*.STL
├── src/{robot_msgs, robot_joint_controller}/   # Gazebo 控制器与消息接口
├── scripts/                              # download_inference_runtime / download_mujoco / convert_policy 等
└── docker/                               # ROS2 Humble + Gazebo + MuJoCo 容器
```

## 环境准备

```bash
# Ubuntu
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
                 libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev
```

并安装下面之一(或两者都装):

- **ROS Noetic**(Ubuntu 20.04) → 走 catkin 编译,使用 `rl_sim`
- **ROS2 Foxy / Humble**(Ubuntu 20.04 / 22.04) → 走 colcon 编译,使用 `rl_sim`

非 ROS 模式只能跑 MuJoCo 仿真(`rl_sim_mujoco`),不需要 ROS。

克隆仓库:

```bash
git clone --recursive --depth 1 git@github.com:SAIKi0125/deploy_UIKA.git
cd deploy_UIKA
git checkout UIKA
```

> 拉子模块需要 SSH key 已加到 GitHub。SDK 子模块在 `src/rl_sar/library/thirdparty/` 下。

## 编译

统一通过 `./build.sh`:

```bash
./build.sh                # ROS 模式构建全部包(需要先 source ROS 环境)
./build.sh rl_sar         # ROS 模式仅构建 rl_sar 包
./build.sh -m  | --cmake  # 纯 CMake(无 ROS),输出到 cmake_build/{bin,lib}
./build.sh -mj | --mujoco # CMake + MuJoCo 仿真支持
./build.sh -c  | --clean  # 清理 build/ cmake_build/ devel/ install/ log/ .catkin_tools/ 与符号链接
```

首次构建会自动通过 `scripts/download_inference_runtime.sh` 下载 libtorch 与(非 Jetson 平台的)onnxruntime,通过 `scripts/download_mujoco.sh` 下载 MuJoCo;`uika_description` 已随仓库提供,不需要再执行 `download_robot_descriptions.sh`。

ROS1 / ROS2 切换会自动清理对方残留;若识别异常,跑一次 `./build.sh -c` 再重新编译。

## 运行

### Gazebo 仿真(ROS)

```bash
# ROS1
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=uika
# 另开终端
rosrun rl_sar rl_sim

# ROS2
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=uika
# 另开终端
ros2 run rl_sar rl_sim
```

> Gazebo 启动后必须立即在第二个终端拉起 `rl_sim`,否则机器人会直接趴下。

### MuJoCo 仿真(无 ROS)

```bash
./build.sh -mj
./cmake_build/bin/rl_sim_mujoco uika scene
```

参数 1 是机器人名(`uika`),参数 2 是 `src/rl_sar_zoo/uika_description/mjcf/<scene>.xml` 中的场景名(默认提供 `scene`)。

### 实机部署

当前仓库中没有 UIKA 专用的 `rl_real_uika.cpp`。两种思路:

1. 把通用 `rl_sim` 接到实机的 ROS 控制接口上(joint_state / joint_command 复用消息定义)
2. 参照原版 `rl_sar` 的 `rl_real_*.cpp`(已在本分支删除,可在上游或提交历史 `7a172ae` 之前的 main 上找到模板)新增 `src/rl_sar/src/rl_real_uika.cpp`,在 `CMakeLists.txt` 中按 `rl_real_*` 模式注册可执行文件

实机入口完成前,请只在仿真中运行。

## 策略与训练仓库的对应关系

| 文件 | 来源 | 含义 |
|---|---|---|
| `policy/uika/himloco/policy.pt` | `himloco_lab/logs/himloco_rsl_rl/uika/<run>/exported/policy.pt` | TorchScript 策略 |
| `policy/uika/himloco/config.yaml` | 与 `himloco_lab` 训练时的 `params/deploy.yaml` 对齐 | 观测/动作/命令归一化、限幅、关节映射 |
| `policy/uika/base.yaml` | UIKA 物理参数 | 关节名/顺序、默认站立角、扭矩上限、`joint_mapping` |

观测维度 = `commands(3) + ang_vel(3) + gravity(3) + dof_pos(12) + dof_vel(12) + actions(12) = 45`,HimLoco encoder 用 6 帧历史,实际推理输入 `45 × 6 = 270`。命令向量 `[lin_vel_x, lin_vel_y, ang_vel_z]`,默认限幅 `[-1, 1]`。

`src/rl_sar/test/test_uika_integration.py` 会把 `policy/uika/` 下的 `config.yaml` / `base.yaml` 与 `himloco_lab` 训练 run 中的 `params/deploy.yaml` 比对,任何关节顺序、缩放、限幅、`default_dof_pos` 漂移都会被它抓出来。导入新策略后请运行此测试。

## 部署完整流程

1. **训练**:在 `himloco_lab` 中训练 UIKA 策略,任务名 `UIKA-Velocity`(详见训练仓库 README)
2. **导出**:`python scripts/himloco_rsl_rl/play.py --task UIKA-Velocity-Play`,产物在 `logs/himloco_rsl_rl/uika/<timestamp>/exported/`
3. **同步**:把 `policy.pt` 拷到 `policy/uika/himloco/policy.pt`,把训练 run 的 `params/deploy.yaml` 中关键字段(`joint_ids_map`、`actions.JointPositionAction.scale/clip`、`commands.base_velocity.ranges`、`observations.*.scale`)同步进 `policy/uika/himloco/config.yaml`
4. **校验**:`python -m unittest src/rl_sar/test/test_uika_integration.py`,确保 deploy 配置与训练导出一致
5. **Sim2Sim**:先用 MuJoCo (`rl_sim_mujoco uika scene`) 验证策略,再上 Gazebo
6. **参数辨识**:用 [PACE](https://github.com/leggedrobotics/pace-sim2real) 校准质量/质心/惯量/阻尼/PD,把校准后的参数回写训练侧再导出新策略
7. **Sim2Real**:在低速、限幅、有人保护、急停就位的条件下首次上机,观察站立角、关节方向、扭矩,确认无误后再放开速度命令

## FSM 状态与按键

`fsm_uika.hpp` 注册了 4 个状态,运行时通过键盘或手柄切换:

| 状态 | 键盘 | 手柄 | 说明 |
|---|---|---|---|
| `RLFSMStatePassive` | `P` | `LB+X` | 阻尼模式,关节零扭矩 + 阻尼,防摔 |
| `RLFSMStateGetUp` | `0` | `A` | 站立(从 Passive 进入会先经预备姿态再到 default_dof_pos) |
| `RLFSMStateGetDown` | `9` | `B` | 蹲下回到起始姿态 |
| `RLFSMStateRLHimLoco` | `1` | `RB+DPadUp` | 加载 `policy/uika/himloco/`,RL 策略接管行走 |

通用按键(`Input::Keyboard` / `Input::Gamepad`)定义在 `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`,UIKA 复用其中的 W/S/A/D/Q/E 与左摇杆作为速度命令输入。

## 参考项目

- [himloco_lab](https://github.com/SAIKi0125/UIKA_lab) ── UIKA 训练仓库(Isaac Lab + HimLoco)
- [rl_sar](https://github.com/fan-ziqi/rl_sar) ── 上游 Sim2Real 部署框架
- [HimLoco](https://github.com/RoboLoco/HimLoco) ── HimLoco 算法原始实现
- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/)
- [robot_lab](https://github.com/fan-ziqi/robot_lab) ── 奖励设计参考
- [PACE Sim2Real](https://github.com/leggedrobotics/pace-sim2real) ── 参数辨识

## 致谢

感谢 [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar)、[IsaacZH/himloco_lab](https://github.com/IsaacZH/himloco_lab) 等上游项目的开源贡献。本仓库为华东理工大学 Robocon 无贰战队 UIKA 仿生足式比赛部署使用,遵循 Apache-2.0 协议。
