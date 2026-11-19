#pragma once

#include "RealtimeMotionBuffer.hpp" // Both controllers expose the same InterpolatedFrame.
#include "VlaPosePacket.hpp"
#include <atomic>
#include <deque>
#include <future>
#include <mutex>
#include <thread>

namespace RLController
{
    struct VlaMotionWindow
    {
        bool ready = false;
        // A latched reference fault holds a single pose over all five steps.
        // The existing tracker and the operator's exit controls continue.
        bool fault = false;
        const char *fault_reason = nullptr;
        std::array<InterpolatedFrame, 5> frames;
        int64_t last_frame_index = -1;
        int64_t receive_age_ns = -1;
    };

    // One consumer (the SuperMimic FSM), one pose publisher per FSM entry.
    class RealtimeMotionBufferVla
    {
    public:
        RealtimeMotionBufferVla(const std::string &endpoint, int hwm,
                                int64_t dt_ns, int64_t tolerance_ns,
                                const Eigen::VectorXf &default_joint_pos,
                                const Eigen::Vector3f &default_base_pos,
                                const Eigen::Quaternionf &default_base_quat,
                                int64_t stale_ns);
        ~RealtimeMotionBufferVla();
        VlaMotionWindow sample();

    private:
        friend struct VlaMotionBufferTest;
        RealtimeMotionBufferVla(int64_t dt_ns, int64_t tolerance_ns,
                                const Eigen::VectorXf &default_joint_pos,
                                const Eigen::Vector3f &default_base_pos,
                                const Eigen::Quaternionf &default_base_quat,
                                int64_t stale_ns);
        void receive(std::string endpoint, int hwm, std::promise<void> started);
        void accept_locked(const VlaPosePacket &packet, int64_t now);
        VlaMotionWindow sample_locked(int64_t now);
        void check_age_locked(int64_t now);

        // Relative integer phase. One VLA frame = 1e9 units, one ns = 30 units.
        // Relative indexing also works when subscribing late to a long session.
        static constexpr int64_t kFrame = 1'000'000'000;
        static constexpr int64_t kHz = 30;
        static constexpr size_t kCapacity = 128;
        const int64_t dt_ns_, stale_ns_;
        int64_t delay_units_;
        InterpolatedFrame held_frame_;
        std::deque<VlaPoseFrame> frames_;
        int64_t cursor_ = 0;
        int64_t last_recv_ns_ = -1, last_sample_ns_ = -1;
        std::string stream_id_;
        const char *fault_ = nullptr;
        std::mutex mutex_;
        std::atomic<bool> stop_{false};
        std::thread receiver_;
    };
}
