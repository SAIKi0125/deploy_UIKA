/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CATCHBALL_PERCEPTION_HPP
#define CATCHBALL_PERCEPTION_HPP

#include "vector_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

struct CatchballPerceptionConfig
{
    std::array<float, 3> cam_offset_base = {0.0f, 0.0f, 0.20f};
    float tan_hfov_half = 1.2109424f;  // tan(100.9 deg / 2)
    float tan_vfov_half = 0.6808758f;  // tan(68.5 deg / 2)
    float dist_min = 0.5f;
    float dist_max = 4.0f;

    float noise_x_base = 0.005f;
    float noise_x_lin = 0.020f;
    float noise_x_quad = 0.005f;
    float noise_x_min = 0.005f;
    float noise_x_max = 0.120f;

    float noise_y_base = 0.001f;
    float noise_y_lin = 0.008f;
    float noise_y_min = 0.002f;
    float noise_y_max = 0.040f;

    float noise_z_base = 0.005f;
    float noise_z_lin = 0.040f;
    float noise_z_min = 0.005f;
    float noise_z_max = 0.200f;

    int latency_steps = 1;
    bool enable_noise = true;
    bool enable_fov = true;
    bool enable_latency = true;
    unsigned int noise_seed = 0;
};

class CatchballPerception
{
public:
    explicit CatchballPerception(const CatchballPerceptionConfig& config = CatchballPerceptionConfig())
        : config_(config), rng_(config.noise_seed)
    {
    }

    std::vector<float> ComputeBallPosB(
        const std::vector<float>& base_pos_w,
        const std::vector<float>& base_quat_wxyz,
        const std::vector<float>& ball_pos_w,
        bool in_flight)
    {
        if (base_pos_w.size() != 3 || base_quat_wxyz.size() != 4 || ball_pos_w.size() != 3)
        {
            throw std::runtime_error("CatchballPerception expects base_pos_w(3), base_quat_wxyz(4), ball_pos_w(3)");
        }

        const std::vector<float> ball_delta_w = {
            ball_pos_w[0] - base_pos_w[0],
            ball_pos_w[1] - base_pos_w[1],
            ball_pos_w[2] - base_pos_w[2],
        };

        const std::vector<float> ball_b_true = QuatRotateInverse(base_quat_wxyz, ball_delta_w);
        std::vector<float> ball_b = ApplyLatency(ball_b_true);

        const std::vector<float> p_cam = {
            ball_b[0] - config_.cam_offset_base[0],
            ball_b[1] - config_.cam_offset_base[1],
            ball_b[2] - config_.cam_offset_base[2],
        };
        const float d = Norm(p_cam);

        const bool visible = !config_.enable_fov || IsVisibleInTrainingCamera(p_cam, d);

        if (config_.enable_noise)
        {
            ApplyDistanceNoise(ball_b, d);
        }

        if (!visible || !in_flight)
        {
            return {0.0f, 0.0f, 0.0f};
        }
        return ball_b;
    }

    void ResetLatency()
    {
        latency_buffer_.clear();
    }

    std::vector<std::array<float, 3>> GetFovCornersBase() const
    {
        std::vector<std::array<float, 3>> corners;
        corners.reserve(8);

        const auto append_corners_at_distance = [this, &corners](float distance)
        {
            const float ray_norm = std::sqrt(
                config_.tan_hfov_half * config_.tan_hfov_half +
                config_.tan_vfov_half * config_.tan_vfov_half +
                1.0f);
            const float z = distance / ray_norm;
            const float x = config_.tan_hfov_half * z;
            const float y = config_.tan_vfov_half * z;

            corners.push_back({
                config_.cam_offset_base[0] - x,
                config_.cam_offset_base[1] - y,
                config_.cam_offset_base[2] + z,
            });
            corners.push_back({
                config_.cam_offset_base[0] + x,
                config_.cam_offset_base[1] - y,
                config_.cam_offset_base[2] + z,
            });
            corners.push_back({
                config_.cam_offset_base[0] + x,
                config_.cam_offset_base[1] + y,
                config_.cam_offset_base[2] + z,
            });
            corners.push_back({
                config_.cam_offset_base[0] - x,
                config_.cam_offset_base[1] + y,
                config_.cam_offset_base[2] + z,
            });
        };

        append_corners_at_distance(config_.dist_min);
        append_corners_at_distance(config_.dist_max);
        return corners;
    }

private:
    std::vector<float> ApplyLatency(const std::vector<float>& current)
    {
        if (!config_.enable_latency || config_.latency_steps <= 0)
        {
            return current;
        }

        if (latency_buffer_.size() != static_cast<size_t>(config_.latency_steps))
        {
            latency_buffer_.assign(static_cast<size_t>(config_.latency_steps), current);
            return current;
        }

        std::vector<float> delayed = latency_buffer_.front();
        for (size_t i = 1; i < latency_buffer_.size(); ++i)
        {
            latency_buffer_[i - 1] = latency_buffer_[i];
        }
        latency_buffer_.back() = current;
        return delayed;
    }

    bool IsVisibleInTrainingCamera(const std::vector<float>& p_cam, float d) const
    {
        const float x_c = p_cam[0];
        const float y_c = p_cam[1];
        const float z_c = p_cam[2];
        return z_c > 0.0f
            && std::fabs(x_c) <= config_.tan_hfov_half * z_c
            && std::fabs(y_c) <= config_.tan_vfov_half * z_c
            && d >= config_.dist_min
            && d <= config_.dist_max;
    }

    void ApplyDistanceNoise(std::vector<float>& ball_b, float d)
    {
        const float sx = Clamp(config_.noise_x_base + config_.noise_x_lin * d + config_.noise_x_quad * d * d,
                               config_.noise_x_min, config_.noise_x_max);
        const float sy = Clamp(config_.noise_y_base + config_.noise_y_lin * d,
                               config_.noise_y_min, config_.noise_y_max);
        const float sz = Clamp(config_.noise_z_base + config_.noise_z_lin * d,
                               config_.noise_z_min, config_.noise_z_max);

        ball_b[0] += SampleNormal(sx);
        ball_b[1] += SampleNormal(sy);
        ball_b[2] += SampleNormal(sz);
    }

    float SampleNormal(float sigma)
    {
        std::normal_distribution<float> distribution(0.0f, sigma);
        return distribution(rng_);
    }

    static float Clamp(float value, float lower, float upper)
    {
        return std::max(lower, std::min(value, upper));
    }

    static float Norm(const std::vector<float>& v)
    {
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    CatchballPerceptionConfig config_;
    std::mt19937 rng_;
    std::vector<std::vector<float>> latency_buffer_;
};

#endif // CATCHBALL_PERCEPTION_HPP
