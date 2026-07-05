/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_UIKA_HPP
#define RL_REAL_UIKA_HPP

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/motor_command12.hpp"
#include "interfaces/msg/motor_feedback12.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <array>
#include <mutex>
#include <string>
#include <vector>

class RL_Real_UIKA : public RL
{
public:
    RL_Real_UIKA(int argc, char **argv);
    ~RL_Real_UIKA();

    std::shared_ptr<rclcpp::Node> ros2_node;

private:
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    void MotorFeedbackCallback(const interfaces::msg::MotorFeedback12::SharedPtr msg);
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void XboxVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    bool SensorsReady() const;
    float ToJointPosition(int index, float motor_position) const;
    float ToJointVelocity(int index, float motor_velocity) const;
    float ToJointTorque(int index, float motor_torque) const;
    float ToMotorPosition(int index, float joint_position) const;
    float ToMotorVelocity(int index, float joint_velocity) const;
    float ToMotorTorque(int index, float joint_torque) const;
    float ToMotorGain(int index, float joint_gain) const;

    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;

    rclcpp::Publisher<interfaces::msg::MotorCommand12>::SharedPtr motor_command_publisher;
    rclcpp::Subscription<interfaces::msg::MotorFeedback12>::SharedPtr motor_feedback_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_subscriber;

    mutable std::mutex state_mutex;
    RobotState<float> latest_state;
    geometry_msgs::msg::Twist cmd_vel;
    bool motor_feedback_received = false;
    bool imu_received = false;

    std::string motor_command_topic = "/motor_command";
    std::string motor_feedback_topic = "/motor_feedback";
    std::string imu_topic = "/imu/data";
    std::string cmd_vel_topic = "/cmd_vel";
    std::string xbox_vel_topic = "/xbox_vel";
    // Jetson rs00_motor handles the 28/15 calf reduction on /motor_command
    // and converts /motor_feedback back to joint-side units.
    float calf_gear_ratio = 1.0f;

    static constexpr std::array<int, 4> calf_indices = {2, 5, 8, 11};
};

#endif // RL_REAL_UIKA_HPP
