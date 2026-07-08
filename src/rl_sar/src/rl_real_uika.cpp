/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_uika.hpp"

#include "logger.hpp"
#include "vector_math.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{

using MotorCommand12 = interfaces::msg::MotorCommand12;
using MotorCommand = interfaces::msg::MotorCommand;
using MotorFeedback12 = interfaces::msg::MotorFeedback12;
using MotorFeedback = interfaces::msg::MotorFeedback;

const std::array<MotorCommand MotorCommand12::*, 12> kCommandFields = {{
    &MotorCommand12::fl_hip, &MotorCommand12::fl_thigh, &MotorCommand12::fl_calf,
    &MotorCommand12::fr_hip, &MotorCommand12::fr_thigh, &MotorCommand12::fr_calf,
    &MotorCommand12::rl_hip, &MotorCommand12::rl_thigh, &MotorCommand12::rl_calf,
    &MotorCommand12::rr_hip, &MotorCommand12::rr_thigh, &MotorCommand12::rr_calf,
}};

const std::array<MotorFeedback MotorFeedback12::*, 12> kFeedbackFields = {{
    &MotorFeedback12::fl_hip, &MotorFeedback12::fl_thigh, &MotorFeedback12::fl_calf,
    &MotorFeedback12::fr_hip, &MotorFeedback12::fr_thigh, &MotorFeedback12::fr_calf,
    &MotorFeedback12::rl_hip, &MotorFeedback12::rl_thigh, &MotorFeedback12::rl_calf,
    &MotorFeedback12::rr_hip, &MotorFeedback12::rr_thigh, &MotorFeedback12::rr_calf,
}};

} // namespace

RL_Real_UIKA::RL_Real_UIKA(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    this->ros2_node = std::make_shared<rclcpp::Node>("rl_real_uika_node");
    this->ang_vel_axis = "body";

    this->ros2_node->declare_parameter<std::string>("robot_name", "uika");
    this->ros2_node->declare_parameter<std::string>("motor_command_topic", this->motor_command_topic);
    this->ros2_node->declare_parameter<std::string>("motor_feedback_topic", this->motor_feedback_topic);
    this->ros2_node->declare_parameter<std::string>("imu_topic", this->imu_topic);
    this->ros2_node->declare_parameter<std::string>("cmd_vel_topic", this->cmd_vel_topic);
    this->ros2_node->declare_parameter<std::string>("xbox_vel_topic", this->xbox_vel_topic);
    this->ros2_node->declare_parameter<std::string>("joy_topic", this->joy_topic);
    this->ros2_node->declare_parameter<double>("calf_gear_ratio", this->calf_gear_ratio);

    this->robot_name = this->ros2_node->get_parameter("robot_name").as_string();
    this->motor_command_topic = this->ros2_node->get_parameter("motor_command_topic").as_string();
    this->motor_feedback_topic = this->ros2_node->get_parameter("motor_feedback_topic").as_string();
    this->imu_topic = this->ros2_node->get_parameter("imu_topic").as_string();
    this->cmd_vel_topic = this->ros2_node->get_parameter("cmd_vel_topic").as_string();
    this->xbox_vel_topic = this->ros2_node->get_parameter("xbox_vel_topic").as_string();
    this->joy_topic = this->ros2_node->get_parameter("joy_topic").as_string();
    this->calf_gear_ratio = static_cast<float>(this->ros2_node->get_parameter("calf_gear_ratio").as_double());

    this->ReadYaml(this->robot_name, "base.yaml");

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

    const int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->InitJointNum(num_of_dofs);
    this->InitOutputs();
    this->InitControl();
    this->latest_state.motor_state.resize(num_of_dofs);

    this->motor_command_publisher = this->ros2_node->create_publisher<MotorCommand12>(
        this->motor_command_topic, rclcpp::SystemDefaultsQoS());
    this->motor_feedback_subscriber = this->ros2_node->create_subscription<MotorFeedback12>(
        this->motor_feedback_topic, rclcpp::SystemDefaultsQoS(),
        [this](const MotorFeedback12::SharedPtr msg) { this->MotorFeedbackCallback(msg); });
    this->imu_subscriber = this->ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        this->imu_topic, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->ImuCallback(msg); });
    this->cmd_vel_subscriber = this->ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        this->cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->CmdVelCallback(msg); });
    this->xbox_vel_subscriber = this->ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        this->xbox_vel_topic, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->XboxVelCallback(msg); });
    this->joy_subscriber = this->ros2_node->create_subscription<sensor_msgs::msg::Joy>(
        this->joy_topic, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) { this->JoyCallback(msg); });

    this->loop_control = std::make_shared<LoopFunc>(
        "loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real_UIKA::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>(
        "loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
        std::bind(&RL_Real_UIKA::RunModel, this));
    this->loop_keyboard = std::make_shared<LoopFunc>(
        "loop_keyboard", 0.05, std::bind(&RL_Real_UIKA::KeyboardInterface, this));

    this->loop_control->start();
    this->loop_rl->start();
    this->loop_keyboard->start();

    std::cout << LOGGER::INFO << "RL_Real_UIKA start" << std::endl;
    std::cout << LOGGER::INFO << "Feedback: " << this->motor_feedback_topic
              << ", IMU: " << this->imu_topic
              << ", command: " << this->motor_command_topic
              << ", joy: " << this->joy_topic
              << ", calf gear: " << this->calf_gear_ratio << std::endl;
}

RL_Real_UIKA::~RL_Real_UIKA()
{
    if (this->loop_keyboard) this->loop_keyboard->shutdown();
    if (this->loop_control) this->loop_control->shutdown();
    if (this->loop_rl) this->loop_rl->shutdown();
    std::cout << LOGGER::INFO << "RL_Real_UIKA exit" << std::endl;
}

bool RL_Real_UIKA::SensorsReady() const
{
    std::lock_guard<std::mutex> lock(this->state_mutex);
    return this->motor_feedback_received && this->imu_received;
}

float RL_Real_UIKA::ToJointPosition(int index, float motor_position) const
{
    if (std::find(calf_indices.begin(), calf_indices.end(), index) != calf_indices.end())
    {
        return motor_position / this->calf_gear_ratio;
    }
    return motor_position;
}

float RL_Real_UIKA::ToJointVelocity(int index, float motor_velocity) const
{
    if (std::find(calf_indices.begin(), calf_indices.end(), index) != calf_indices.end())
    {
        return motor_velocity / this->calf_gear_ratio;
    }
    return motor_velocity;
}

float RL_Real_UIKA::ToJointTorque(int index, float motor_torque) const
{
    if (std::find(calf_indices.begin(), calf_indices.end(), index) != calf_indices.end())
    {
        return motor_torque * this->calf_gear_ratio;
    }
    return motor_torque;
}

float RL_Real_UIKA::ToMotorPosition(int index, float joint_position) const
{
    if (std::find(calf_indices.begin(), calf_indices.end(), index) != calf_indices.end())
    {
        return joint_position * this->calf_gear_ratio;
    }
    return joint_position;
}

float RL_Real_UIKA::ToMotorVelocity(int index, float joint_velocity) const
{
    if (std::find(calf_indices.begin(), calf_indices.end(), index) != calf_indices.end())
    {
        return joint_velocity * this->calf_gear_ratio;
    }
    return joint_velocity;
}

float RL_Real_UIKA::ToMotorTorque(int index, float joint_torque) const
{
    if (std::find(calf_indices.begin(), calf_indices.end(), index) != calf_indices.end())
    {
        return joint_torque / this->calf_gear_ratio;
    }
    return joint_torque;
}

float RL_Real_UIKA::ToMotorGain(int index, float joint_gain) const
{
    if (std::find(calf_indices.begin(), calf_indices.end(), index) != calf_indices.end())
    {
        return joint_gain / (this->calf_gear_ratio * this->calf_gear_ratio);
    }
    return joint_gain;
}

void RL_Real_UIKA::MotorFeedbackCallback(const MotorFeedback12::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(this->state_mutex);
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const auto &feedback = msg.get()->*kFeedbackFields[i];
        this->latest_state.motor_state.q[i] = ToJointPosition(i, static_cast<float>(feedback.position));
        this->latest_state.motor_state.dq[i] = ToJointVelocity(i, static_cast<float>(feedback.velocity));
        this->latest_state.motor_state.tau_est[i] = ToJointTorque(i, static_cast<float>(feedback.torque));
    }
    this->motor_feedback_received = true;
}

void RL_Real_UIKA::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(this->state_mutex);
    this->latest_state.imu.quaternion = {
        static_cast<float>(msg->orientation.w),
        static_cast<float>(msg->orientation.x),
        static_cast<float>(msg->orientation.y),
        static_cast<float>(msg->orientation.z)
    };
    this->latest_state.imu.gyroscope = {
        static_cast<float>(msg->angular_velocity.x),
        static_cast<float>(msg->angular_velocity.y),
        static_cast<float>(msg->angular_velocity.z)
    };
    this->latest_state.imu.accelerometer = {
        static_cast<float>(msg->linear_acceleration.x),
        static_cast<float>(msg->linear_acceleration.y),
        static_cast<float>(msg->linear_acceleration.z)
    };
    this->imu_received = true;
}

void RL_Real_UIKA::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel = *msg;
}

void RL_Real_UIKA::XboxVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->control.x = static_cast<float>(msg->linear.x);
    this->control.y = static_cast<float>(msg->linear.y);
    this->control.yaw = static_cast<float>(msg->angular.z);
}

void RL_Real_UIKA::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    this->joy_msg = *msg;

    const auto button = [this](size_t index) -> bool {
        return index < this->joy_msg.buttons.size() && this->joy_msg.buttons[index] != 0;
    };
    const auto axis = [this](size_t index) -> float {
        return index < this->joy_msg.axes.size() ? this->joy_msg.axes[index] : 0.0f;
    };

    // Same F710/Xbox mapping used by rl_sim:
    // buttons: A=0, B=1, X=2, Y=3, LB=4, RB=5, stickL=9, stickR=10
    // axes: Lx=0, Ly=1, Rx=3, DPadX=6, DPadY=7
    if (button(0)) this->control.SetGamepad(Input::Gamepad::A);
    if (button(1)) this->control.SetGamepad(Input::Gamepad::B);
    if (button(2)) this->control.SetGamepad(Input::Gamepad::X);
    if (button(3)) this->control.SetGamepad(Input::Gamepad::Y);
    if (button(4)) this->control.SetGamepad(Input::Gamepad::LB);
    if (button(5)) this->control.SetGamepad(Input::Gamepad::RB);
    if (button(9)) this->control.SetGamepad(Input::Gamepad::LStick);
    if (button(10)) this->control.SetGamepad(Input::Gamepad::RStick);
    if (axis(7) > 0.0f) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (axis(7) < 0.0f) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (axis(6) < 0.0f) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (axis(6) > 0.0f) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (button(4) && button(0)) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (button(4) && button(1)) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (button(4) && button(2)) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (button(4) && button(3)) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (button(4) && button(9)) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (button(4) && button(10)) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (button(4) && axis(7) > 0.0f) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (button(4) && axis(7) < 0.0f) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (button(4) && axis(6) < 0.0f) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (button(4) && axis(6) > 0.0f) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (button(5) && button(0)) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (button(5) && button(1)) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (button(5) && button(2)) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (button(5) && button(3)) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (button(5) && button(9)) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (button(5) && button(10)) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (button(5) && axis(7) > 0.0f) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (button(5) && axis(7) < 0.0f) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (button(5) && axis(6) < 0.0f) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (button(5) && axis(6) > 0.0f) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (button(4) && button(5)) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = axis(1);
    this->control.y = axis(0);
    this->control.yaw = axis(3);
}

void RL_Real_UIKA::GetState(RobotState<float> *state)
{
    std::lock_guard<std::mutex> lock(this->state_mutex);
    *state = this->latest_state;
}

void RL_Real_UIKA::SetCommand(const RobotCommand<float> *command)
{
    MotorCommand12 command_msg;
    command_msg.header.stamp = this->ros2_node->now();
    command_msg.header.frame_id = "rl_real_uika";

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        auto &motor_command = command_msg.*kCommandFields[i];
        motor_command.position = ToMotorPosition(i, command->motor_command.q[i]);
        motor_command.velocity = ToMotorVelocity(i, command->motor_command.dq[i]);
        motor_command.torque = ToMotorTorque(i, command->motor_command.tau[i]);
        motor_command.kp = ToMotorGain(i, command->motor_command.kp[i]);
        motor_command.kd = ToMotorGain(i, command->motor_command.kd[i]);
    }

    this->motor_command_publisher->publish(command_msg);
}

void RL_Real_UIKA::RobotControl()
{
    if (!this->SensorsReady())
    {
        RCLCPP_WARN_THROTTLE(
            this->ros2_node->get_logger(), *this->ros2_node->get_clock(), 2000,
            "Waiting for %s and %s", this->motor_feedback_topic.c_str(), this->imu_topic.c_str());
        return;
    }

    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);
}

void RL_Real_UIKA::RunModel()
{
    std::lock_guard<std::mutex> policy_lock(this->policy_mutex);

    if (this->rl_init_done && this->SensorsReady())
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        if (this->obs.commands.size() <= 3)
        {
            this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        }
        if (this->control.navigation_mode)
        {
            this->obs.commands = {
                static_cast<float>(this->cmd_vel.linear.x),
                static_cast<float>(this->cmd_vel.linear.y),
                static_cast<float>(this->cmd_vel.angular.z)
            };
        }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            this->output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            this->output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            this->output_dof_tau_queue.push(this->output_dof_tau);
        }

        this->TorqueProtect(this->output_dof_tau);
        this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);
    }
}

std::vector<float> RL_Real_UIKA::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();
    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() &&
        !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(
            actions,
            this->params.Get<std::vector<float>>("clip_actions_lower"),
            this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    return actions;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto rl_real_uika = std::make_shared<RL_Real_UIKA>(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(rl_real_uika->ros2_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
