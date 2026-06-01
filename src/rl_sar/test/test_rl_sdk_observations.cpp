/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"
#include "catchball_perception.hpp"
#include "fsm_go2w.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

class TestRL : public RL
{
public:
    std::vector<float> Forward() override { return {}; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

void RequireNear(float actual, float expected, const std::string& label)
{
    if (std::fabs(actual - expected) > 1.0e-6f)
    {
        throw std::runtime_error(label + " expected " + std::to_string(expected) + ", got " + std::to_string(actual));
    }
}

void TestBallPositionObservation()
{
    TestRL rl;
    rl.params.config_node = YAML::Load(R"(
observations: ["ball_pos_b"]
clip_obs: 100.0
)");
    rl.obs.ball_pos_b = {1.25f, -0.5f, 0.75f};

    const auto obs = rl.ComputeObservation();
    if (obs.size() != 3)
    {
        throw std::runtime_error("ball_pos_b observation should have size 3");
    }
    RequireNear(obs[0], 1.25f, "ball_pos_b[0]");
    RequireNear(obs[1], -0.5f, "ball_pos_b[1]");
    RequireNear(obs[2], 0.75f, "ball_pos_b[2]");
}

void TestCatchballPerceptionKeepsBaseCoordinateForVisibleBall()
{
    CatchballPerceptionConfig config;
    config.enable_noise = false;
    CatchballPerception perception(config);
    const std::vector<float> base_quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    const std::vector<float> base_pos_w = {0.0f, 0.0f, 0.0f};
    const std::vector<float> ball_pos_w = {0.25f, -0.10f, 1.20f};

    const auto obs = perception.ComputeBallPosB(base_pos_w, base_quat_wxyz, ball_pos_w, true);

    if (obs.size() != 3)
    {
        throw std::runtime_error("catchball perception should output a 3D ball_pos_b");
    }
    RequireNear(obs[0], 0.25f, "visible ball_pos_b[0]");
    RequireNear(obs[1], -0.10f, "visible ball_pos_b[1]");
    RequireNear(obs[2], 1.20f, "visible ball_pos_b[2]");
}

void TestCatchballPerceptionMasksOutsideTrainingFov()
{
    CatchballPerceptionConfig config;
    config.enable_noise = false;
    CatchballPerception perception(config);
    const std::vector<float> base_quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    const std::vector<float> base_pos_w = {0.0f, 0.0f, 0.0f};

    const auto below_camera = perception.ComputeBallPosB(base_pos_w, base_quat_wxyz, {0.0f, 0.0f, 0.10f}, true);
    const auto too_close = perception.ComputeBallPosB(base_pos_w, base_quat_wxyz, {0.0f, 0.0f, 0.60f}, true);
    const auto outside_hfov = perception.ComputeBallPosB(base_pos_w, base_quat_wxyz, {2.0f, 0.0f, 1.0f}, true);
    const auto not_in_flight = perception.ComputeBallPosB(base_pos_w, base_quat_wxyz, {0.0f, 0.0f, 1.20f}, false);

    for (const auto& obs : {below_camera, too_close, outside_hfov, not_in_flight})
    {
        RequireNear(obs[0], 0.0f, "masked ball_pos_b[0]");
        RequireNear(obs[1], 0.0f, "masked ball_pos_b[1]");
        RequireNear(obs[2], 0.0f, "masked ball_pos_b[2]");
    }
}

void TestCatchballPerceptionFovCornersUseTrainingCameraGeometry()
{
    CatchballPerceptionConfig config;
    config.cam_offset_base = {0.0f, 0.0f, 0.20f};
    config.tan_hfov_half = 1.0f;
    config.tan_vfov_half = 0.5f;
    config.dist_min = 1.5f;
    config.dist_max = 3.0f;
    CatchballPerception perception(config);

    const auto corners = perception.GetFovCornersBase();

    if (corners.size() != 8)
    {
        throw std::runtime_error("FOV corner visualization should expose near and far rectangles");
    }
    RequireNear(corners[0][0], -1.0f, "near lower-left x");
    RequireNear(corners[0][1], -0.5f, "near lower-left y");
    RequireNear(corners[0][2], 1.20f, "near lower-left z with camera offset");
    RequireNear(corners[7][0], -2.0f, "far upper-left x");
    RequireNear(corners[7][1], 1.0f, "far upper-left y");
    RequireNear(corners[7][2], 2.20f, "far upper-left z with camera offset");
}

void TestInitObservationsUsesWxyzIdentityQuaternion()
{
    TestRL rl;
    rl.params.config_node = YAML::Load(R"(
num_of_dofs: 1
default_dof_pos: [0.0]
commands_scale: []
observations: []
clip_obs: 100.0
)");

    rl.InitObservations();

    RequireNear(rl.obs.base_quat[0], 1.0f, "base_quat[w]");
    RequireNear(rl.obs.base_quat[1], 0.0f, "base_quat[x]");
    RequireNear(rl.obs.base_quat[2], 0.0f, "base_quat[y]");
    RequireNear(rl.obs.base_quat[3], 0.0f, "base_quat[z]");
}

void TestStateControllerClampsCommandsToTrainingRange()
{
    TestRL rl;
    rl.params.config_node = YAML::Load(R"(
commands_limit_lower: [-0.5, -1.0, -1.0]
commands_limit_upper: [0.5, 1.0, 1.0]
)");

    RobotState<float> state;
    RobotCommand<float> command;

    rl.control.x = 0.5f;
    rl.control.y = 1.0f;
    rl.control.yaw = 1.0f;

    rl.control.SetKeyboard(Input::Keyboard::W);
    rl.StateController(&state, &command);
    RequireNear(rl.control.x, 0.5f, "clamped command x upper");

    rl.control.SetKeyboard(Input::Keyboard::A);
    rl.StateController(&state, &command);
    RequireNear(rl.control.y, 1.0f, "clamped command y upper");

    rl.control.SetKeyboard(Input::Keyboard::Q);
    rl.StateController(&state, &command);
    RequireNear(rl.control.yaw, 1.0f, "clamped command yaw upper");
}

void TestManualCatchballTargetUsesBaseFluAxes()
{
    const std::vector<float> init = {1.0f, 0.0f, 0.8f};
    const std::vector<float> min = {0.3f, -1.5f, 0.3f};
    const std::vector<float> max = {2.5f, 1.5f, 2.5f};

    auto target = go2w_fsm::UpdateManualBallTarget(
        init, Input::Keyboard::W, 0.05f, init, min, max);
    RequireNear(target[0], 1.05f, "W increments base x");
    RequireNear(target[1], 0.0f, "W keeps base y");
    RequireNear(target[2], 0.8f, "W keeps base z");

    target = go2w_fsm::UpdateManualBallTarget(
        target, Input::Keyboard::A, 0.05f, init, min, max);
    RequireNear(target[0], 1.05f, "A keeps base x");
    RequireNear(target[1], 0.05f, "A increments base y");
    RequireNear(target[2], 0.8f, "A keeps base z");

    target = go2w_fsm::UpdateManualBallTarget(
        target, Input::Keyboard::Q, 0.05f, init, min, max);
    RequireNear(target[0], 1.05f, "Q keeps base x");
    RequireNear(target[1], 0.05f, "Q keeps base y");
    RequireNear(target[2], 0.85f, "Q increments base z");

    target = go2w_fsm::UpdateManualBallTarget(
        target, Input::Keyboard::Space, 0.05f, init, min, max);
    RequireNear(target[0], init[0], "Space resets base x");
    RequireNear(target[1], init[1], "Space resets base y");
    RequireNear(target[2], init[2], "Space resets base z");
}

} // namespace

int main()
{
    TestBallPositionObservation();
    TestCatchballPerceptionKeepsBaseCoordinateForVisibleBall();
    TestCatchballPerceptionMasksOutsideTrainingFov();
    TestCatchballPerceptionFovCornersUseTrainingCameraGeometry();
    TestInitObservationsUsesWxyzIdentityQuaternion();
    TestStateControllerClampsCommandsToTrainingRange();
    TestManualCatchballTargetUsesBaseFluAxes();
    std::cout << "RL SDK observation tests passed" << std::endl;
    return 0;
}
