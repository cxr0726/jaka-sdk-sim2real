#pragma once
// Deterministic legacy API substitute for actual FSM control-flow tests only.
// The separate legacy-buffer test compiles the unmodified production receiver.
#include <array>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace RLController
{
    struct InterpolatedFrame
    {
        Eigen::Vector3f root_pos_w{0, 0, .65f};
        Eigen::Quaternionf root_quat_w = Eigen::Quaternionf::Identity();
        Eigen::VectorXf dof_pos = Eigen::VectorXf::Zero(27);
    };
    class RealtimeMotionBuffer
    {
    public:
        static constexpr int kFutureSteps = 5;
        static inline bool fail_construct = false, fail_query = false, next_ready = false;
        static inline int ready_calls = 0, query_calls = 0;
        static inline std::array<InterpolatedFrame, 5> next{};
        template<class... Args> explicit RealtimeMotionBuffer(Args&&...)
        { if (fail_construct) throw std::runtime_error("injected legacy buffer construction failure"); }
        bool ready() const { ++ready_calls; return next_ready; }
        std::array<InterpolatedFrame, 5> get_obs()
        {
            ++query_calls;
            if (fail_query) throw std::runtime_error("injected legacy query failure");
            return next;
        }
    };
}
