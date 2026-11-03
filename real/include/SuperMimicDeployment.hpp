#pragma once

#include <array>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include "onnxruntime_cxx_api.h"

namespace RLController
{
    // Real deployment deliberately gets ALL joint/control values from YAML.
    // ONNX metadata is never read; the ONNX supplies only the actor/tensor API.
    struct SuperMimicDeployment
    {
        static constexpr int kNumJoints = 27;
        std::array<int, kNumJoints> policy_to_deploy{};
        std::array<int, kNumJoints> policy_from_buffer{};
        std::array<float, kNumJoints> default_joint_pos{};
        std::array<float, kNumJoints> action_scale{};
        float angular_velocity_scale{0.5f};
        float joint_velocity_scale{0.2f};

        SuperMimicDeployment(Ort::Session &session, const YAML::Node &config,
                            const std::vector<std::string> &deploy_names,
                            const std::vector<double> &deploy_default_pos,
                            double fallback_action_scale);
        Eigen::VectorXf buffer_default_joint_pos() const;
    };
}
