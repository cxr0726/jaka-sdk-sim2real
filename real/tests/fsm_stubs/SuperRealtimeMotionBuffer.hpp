#pragma once
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace RLController
{
    struct InterpolatedFrame
    {
        Eigen::Vector3f root_pos_w{0,0,.65f};
        Eigen::Quaternionf root_quat_w=Eigen::Quaternionf::Identity();
        Eigen::VectorXf dof_pos=Eigen::VectorXf::Zero(27);
    };
    struct MotionTimingConfig {};
    struct MotionWindow
    {
        enum class Status {Buffering,Tracking,Underrun,Stale};
        Status status=Status::Buffering;
        uint64_t generation=0;
        std::array<InterpolatedFrame,5> frames{};
        int64_t query_steady_ns=0,latest_recv_age_ns=0;
        bool clamped_low=false,clamped_high=false;
        bool reference_ready()const{return status==Status::Tracking||status==Status::Underrun;}
    };
    // Deterministic input window; deliberately no sockets, clock or thread.
    class SuperRealtimeMotionBuffer
    {
    public:
        static constexpr int kFutureSteps=5;
        static inline bool fail_construct=false,fail_query=false;
        static inline MotionWindow next{};
        template<class... Args> explicit SuperRealtimeMotionBuffer(Args&&...)
        {if(fail_construct)throw std::runtime_error("injected buffer construction failure");}
        MotionWindow get_observation()
        {if(fail_query)throw std::runtime_error("injected query failure");return next;}
        static const char *status_name(MotionWindow::Status s)
        {
            switch(s){case MotionWindow::Status::Buffering:return "buffering";
            case MotionWindow::Status::Tracking:return "tracking";
            case MotionWindow::Status::Underrun:return "underrun";
            default:return "stale";}
        }
    };
}
