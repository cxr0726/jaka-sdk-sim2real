#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "onnxruntime_cxx_api.h"

namespace RLController
{
    // The actor owns the policy joint order. RobotIntf uses the fixed legacy
    // deployment order, and RealtimeMotionBuffer emits legacy IsaacLab order.
    // Keep all three orders explicit instead of reusing the old actor's map.
    struct SuperMimicDeployment
    {
        static constexpr int kNumJoints = 27;

        std::array<int, kNumJoints> policy_to_deploy{};
        std::array<int, kNumJoints> policy_from_buffer{};
        std::array<float, kNumJoints> default_joint_pos{};
        std::array<float, kNumJoints> action_scale{};
        std::array<float, kNumJoints> joint_stiffness{};
        std::array<float, kNumJoints> joint_damping{};
        float angular_velocity_scale{0.5f};
        float joint_velocity_scale{0.2f};

        SuperMimicDeployment(Ort::Session &session,
                            const std::vector<std::string> &deploy_names);

        Eigen::VectorXf buffer_default_joint_pos() const;
    };
} // namespace RLController
