/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim_mujoco.hpp"

#include <cmath>

RL_Sim* RL_Sim::instance = nullptr;

namespace
{

double MujocoActuatorGearRatio(const mjModel* model, int actuator_index)
{
    if (model == nullptr || actuator_index < 0 || actuator_index >= model->nu)
    {
        return 1.0;
    }

    const double gear = model->actuator_gear[actuator_index * 6];
    if (std::abs(gear) < 1.0e-9)
    {
        return 1.0;
    }
    return gear;
}

double MujocoSensorJointPosition(const mjModel* model, const mjData* data, int actuator_index, int sensor_index)
{
    return data->sensordata[sensor_index] / MujocoActuatorGearRatio(model, actuator_index);
}

double MujocoSensorJointVelocity(const mjModel* model, const mjData* data, int actuator_index, int sensor_index)
{
    return data->sensordata[sensor_index] / MujocoActuatorGearRatio(model, actuator_index);
}

double MujocoSensorJointTorque(const mjModel* model, const mjData* data, int actuator_index, int sensor_index)
{
    return data->sensordata[sensor_index] * MujocoActuatorGearRatio(model, actuator_index);
}

std::vector<float> MjBodyPosition(const mjData* data, int body_id)
{
    const mjtNum* pos = data->xpos + 3 * body_id;
    return {
        static_cast<float>(pos[0]),
        static_cast<float>(pos[1]),
        static_cast<float>(pos[2]),
    };
}

std::vector<float> MjBodyQuaternionWxyz(const mjData* data, int body_id)
{
    const mjtNum* quat = data->xquat + 4 * body_id;
    return {
        static_cast<float>(quat[0]),
        static_cast<float>(quat[1]),
        static_cast<float>(quat[2]),
        static_cast<float>(quat[3]),
    };
}

std::array<double, 3> RotateYaw(double yaw, const std::array<double, 3>& point)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {
        c * point[0] - s * point[1],
        s * point[0] + c * point[1],
        point[2],
    };
}

double YawFromQuaternionWxyz(const std::vector<float>& quat)
{
    const double w = quat[0];
    const double x = quat[1];
    const double y = quat[2];
    const double z = quat[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

} // namespace

RL_Sim::RL_Sim(int argc, char **argv)
{
    // Set static instance pointer early for signal handler
    instance = this;

    if (argc < 3)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " robot_name scene_name" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    else
    {
        this->robot_name = argv[1];
        this->scene_name = argv[2];
    }

    this->ang_vel_axis = "body";

    // now launch mujoco
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;

    // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
#endif

    // print version, check compatibility
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/../rl_sar_zoo/" + this->robot_name + "_description/mjcf/" + this->scene_name + ".xml";

    // start physics thread
    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());
    physicsthreadhandle.detach();

    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->mj_model = m;
    this->mj_data = d;
    this->SetupSysJoystick("/dev/input/js0", 16); // 16 bits joystick

    // read params from yaml
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    this->InitCatchballMujoco();

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    // joystick
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetSysJoystick, this));
    this->loop_joystick->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;

    // start simulation UI loop (blocking call)
    sim->RenderLoop();
}

RL_Sim::~RL_Sim()
{
    // Clear static instance pointer
    instance = nullptr;

    this->loop_keyboard->shutdown();
    this->loop_joystick->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            const int actuator_index = this->params.Get<std::vector<int>>("joint_mapping")[i];
            state->motor_state.q[i] = MujocoSensorJointPosition(mj_model, mj_data, actuator_index, actuator_index);
            state->motor_state.dq[i] = MujocoSensorJointVelocity(
                mj_model, mj_data, actuator_index, actuator_index + this->params.Get<int>("num_of_dofs"));
            state->motor_state.tau_est[i] = MujocoSensorJointTorque(
                mj_model, mj_data, actuator_index, actuator_index + 2 * this->params.Get<int>("num_of_dofs"));
        }
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    if (mj_data)
    {
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            const int actuator_index = this->params.Get<std::vector<int>>("joint_mapping")[i];
            const double gear = MujocoActuatorGearRatio(mj_model, actuator_index);
            const double joint_pos = MujocoSensorJointPosition(mj_model, mj_data, actuator_index, actuator_index);
            const double joint_vel = MujocoSensorJointVelocity(
                mj_model, mj_data, actuator_index, actuator_index + this->params.Get<int>("num_of_dofs"));
            const double joint_torque =
                command->motor_command.tau[i] +
                command->motor_command.kp[i] * (command->motor_command.q[i] - joint_pos) +
                command->motor_command.kd[i] * (command->motor_command.dq[i] - joint_vel);
            mj_data->ctrl[actuator_index] = joint_torque / gear;
        }
    }
}

void RL_Sim::RobotControl()
{
    // Lock the sim mutex once for the entire control cycle to prevent race conditions
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    this->GetState(&this->robot_state);
    this->UpdateCatchballThrowMujoco();

    this->StateController(&this->robot_state, &this->robot_command);

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {
            mj_resetData(this->mj_model, this->mj_data);
            mj_forward(this->mj_model, this->mj_data);
            this->InitCatchballMujoco();
        }
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            sim->run = 0;
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            sim->run = 1;
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Sim::InitCatchballMujoco()
{
    if (!this->mj_model || !this->mj_data)
    {
        return;
    }

    this->mujoco_base_body_id = mj_name2id(this->mj_model, mjOBJ_BODY, "base_link");
    this->mujoco_ball_body_id = mj_name2id(this->mj_model, mjOBJ_BODY, "catchball");
    this->mujoco_ball_joint_id = mj_name2id(this->mj_model, mjOBJ_JOINT, "catchball_freejoint");

    if (this->mujoco_ball_body_id < 0 || this->mujoco_ball_joint_id < 0)
    {
        return;
    }

    this->mujoco_ball_qpos_adr = this->mj_model->jnt_qposadr[this->mujoco_ball_joint_id];
    this->mujoco_ball_dof_adr = this->mj_model->jnt_dofadr[this->mujoco_ball_joint_id];
    this->mujoco_next_throw_time = this->mj_data->time + 0.8;
    this->HideCatchballMujoco();
    std::cout << LOGGER::INFO << "[MuJoCo catchball] enabled with body 'catchball'" << std::endl;
}

void RL_Sim::HideCatchballMujoco()
{
    if (this->mujoco_ball_qpos_adr < 0 || this->mujoco_ball_dof_adr < 0)
    {
        return;
    }

    mjtNum* qpos = this->mj_data->qpos + this->mujoco_ball_qpos_adr;
    mjtNum* qvel = this->mj_data->qvel + this->mujoco_ball_dof_adr;
    qpos[0] = 0.0;
    qpos[1] = 0.0;
    qpos[2] = -10.0;
    qpos[3] = 1.0;
    qpos[4] = 0.0;
    qpos[5] = 0.0;
    qpos[6] = 0.0;
    for (int i = 0; i < 6; ++i)
    {
        qvel[i] = 0.0;
    }
    this->mujoco_ball_in_flight = false;
}

void RL_Sim::ThrowCatchballMujoco()
{
    if (this->mujoco_base_body_id < 0 || this->mujoco_ball_qpos_adr < 0 || this->mujoco_ball_dof_adr < 0)
    {
        return;
    }

    const auto base_pos = MjBodyPosition(this->mj_data, this->mujoco_base_body_id);
    const auto base_quat = MjBodyQuaternionWxyz(this->mj_data, this->mujoco_base_body_id);
    const double yaw = YawFromQuaternionWxyz(base_quat);

    // Training-aligned rejection sampling parameters
    constexpr double g = 9.81;
    constexpr double zc_world = 0.6;
    constexpr double vx_max = 2.5;
    constexpr double speed_safety_factor = 0.7;
    constexpr double reaction_margin = 0.35;
    constexpr double t_min = 1.20, t_max = 1.55;
    constexpr double x_min = 0.5, x_cap = 2.0;
    constexpr double vh_min = 0.5, vh_max = 3.5;
    constexpr double z0_world_min = 0.4, z0_world_max = 2.2;
    constexpr double vz_min = 1.5, vz_max = 8.5;
    constexpr double x0_min = -4.0, x0_max = 5.0;
    constexpr double y0_min = -3.5, y0_max = 3.5;
    constexpr double speed_min = 2.0, speed_max = 9.5;
    constexpr double z_apex_max = 5.0;
    constexpr int max_resample = 100;
    constexpr double pi = 3.141592653589793;

    const double vx_eff = speed_safety_factor * vx_max;

    double x0 = 0, y0 = 0, z0 = 0, xc = 0, yc = 0, tc = 0;
    double v0x = 0, v0y = 0, v0z = 0;
    bool found = false;

    std::uniform_real_distribution<double> dist_tc(t_min, t_max);
    std::uniform_real_distribution<double> dist_phi(-pi, pi);
    std::uniform_real_distribution<double> dist_vh(vh_min, vh_max);
    std::uniform_real_distribution<double> dist_theta(-pi, pi);

    for (int i = 0; i < max_resample; ++i)
    {
        tc = dist_tc(this->mujoco_throw_rng);
        const double t_eff = tc - reaction_margin;
        if (t_eff <= 0.0) continue;

        const double r_max = std::min(vx_eff * t_eff, x_cap);
        if (r_max <= x_min) continue;

        std::uniform_real_distribution<double> dist_rc(x_min, r_max);
        const double rc = dist_rc(this->mujoco_throw_rng);
        const double phi = dist_phi(this->mujoco_throw_rng);
        xc = rc * std::cos(phi);
        yc = rc * std::sin(phi);

        const double vh = dist_vh(this->mujoco_throw_rng);
        const double theta = dist_theta(this->mujoco_throw_rng);
        x0 = xc - vh * tc * std::cos(theta);
        y0 = yc - vh * tc * std::sin(theta);

        const double z_low = std::max(z0_world_min, zc_world + 0.5 * g * tc * tc - vz_max * tc);
        const double z_high = std::min(z0_world_max, zc_world + 0.5 * g * tc * tc - vz_min * tc);
        if (z_low >= z_high) continue;

        std::uniform_real_distribution<double> dist_z0(z_low, z_high);
        z0 = dist_z0(this->mujoco_throw_rng);

        v0x = (xc - x0) / tc;
        v0y = (yc - y0) / tc;
        v0z = (zc_world - z0 + 0.5 * g * tc * tc) / tc;

        if (x0 < x0_min || x0 > x0_max) continue;
        if (y0 < y0_min || y0 > y0_max) continue;
        if (z0 < z0_world_min || z0 > z0_world_max) continue;

        const double speed = std::sqrt(v0x * v0x + v0y * v0y + v0z * v0z);
        if (speed < speed_min || speed > speed_max) continue;
        if (v0z < vz_min || v0z > vz_max) continue;

        const double vz_pos = std::max(v0z, 0.0);
        const double z_apex = z0 + vz_pos * vz_pos / (2.0 * g);
        if (z_apex > z_apex_max) continue;

        found = true;
        break;
    }

    if (!found)
    {
        // Fallback
        tc = 1.35;
        z0 = 1.2;
        const double rc = 1.2;
        const double phi = dist_phi(this->mujoco_throw_rng);
        xc = rc * std::cos(phi);
        yc = rc * std::sin(phi);
        x0 = -1.0;
        y0 = 0.0;
        v0x = (xc - x0) / tc;
        v0y = (yc - y0) / tc;
        v0z = (zc_world - z0 + 0.5 * g * tc * tc) / tc;
    }

    // Transform from robot-local to world frame
    const auto p0_world = RotateYaw(yaw, {x0, y0, z0});
    const auto v0_world = RotateYaw(yaw, {v0x, v0y, 0.0});

    mjtNum* qpos = this->mj_data->qpos + this->mujoco_ball_qpos_adr;
    mjtNum* qvel = this->mj_data->qvel + this->mujoco_ball_dof_adr;
    qpos[0] = static_cast<double>(base_pos[0]) + p0_world[0];
    qpos[1] = static_cast<double>(base_pos[1]) + p0_world[1];
    qpos[2] = p0_world[2];
    qpos[3] = 1.0;
    qpos[4] = 0.0;
    qpos[5] = 0.0;
    qpos[6] = 0.0;
    qvel[0] = v0_world[0];
    qvel[1] = v0_world[1];
    qvel[2] = v0z;
    qvel[3] = 0.0;
    qvel[4] = 0.0;
    qvel[5] = 0.0;

    this->mujoco_throw_start_time = this->mj_data->time;
    this->mujoco_ball_in_flight = true;
    mj_forward(this->mj_model, this->mj_data);
}

void RL_Sim::UpdateCatchballThrowMujoco()
{
    if (this->mujoco_ball_body_id < 0 || this->mujoco_ball_qpos_adr < 0)
    {
        return;
    }

    const double now = this->mj_data->time;
    const mjtNum* ball_pos = this->mj_data->xpos + 3 * this->mujoco_ball_body_id;
    if (this->mujoco_ball_in_flight)
    {
        if (ball_pos[2] < 0.05 || now - this->mujoco_throw_start_time > 3.0)
        {
            this->HideCatchballMujoco();
            this->mujoco_next_throw_time = now + 0.8;
        }
        return;
    }

    if (now >= this->mujoco_next_throw_time)
    {
        this->ThrowCatchballMujoco();
    }
}

void RL_Sim::UpdateCatchballObservationMujoco()
{
    if (this->manual_ball_pos_override_enabled || this->mujoco_base_body_id < 0 || this->mujoco_ball_body_id < 0)
    {
        return;
    }

    const auto base_pos = MjBodyPosition(this->mj_data, this->mujoco_base_body_id);
    const auto base_quat = MjBodyQuaternionWxyz(this->mj_data, this->mujoco_base_body_id);
    const auto ball_pos = MjBodyPosition(this->mj_data, this->mujoco_ball_body_id);
    const bool in_flight = this->mujoco_ball_in_flight && ball_pos[2] >= 0.05f;

    this->obs.ball_pos_b = this->catchball_perception.ComputeBallPosB(base_pos, base_quat, ball_pos, in_flight);
}

void RL_Sim::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        // exit(1);
    }

    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Sim::GetSysJoystick()
{
    // Clear all button event states
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    // Check if joystick is valid before using
    if (!this->sys_js)
    {
        return;
    }

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
            {
                this->sys_js_axis[this->sys_js_event.number] = 0;
            }
            else
            {
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
            }
        }
    }

    if (this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);

    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        this->control.x = ly;
        this->control.y = lx;
        this->control.yaw = rx;
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }
}

void RL_Sim::RunModel()
{
    std::lock_guard<std::mutex> policy_lock(this->policy_mutex);

    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        if (this->config_name != "takeoff")
        {
            this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        }
        //not currently available for non-ros mujoco version
        // if (this->control.navigation_mode)
        // {
        //     this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        // }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;
        if (this->config_name == "catchball")
        {
            const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
            this->UpdateCatchballObservationMujoco();
        }

        this->obs.actions = this->ApplyActionFilter(this->Forward());
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mj_data->sensordata[i]);
        // this->plot_target_joint_pos[i].push_back();  // TODO
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Sim::instance && RL_Sim::instance->sim)
    {
        RL_Sim::instance->sim->exitrequest.store(1);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    RL_Sim rl_sar(argc, argv);
    return 0;
}
