#ifndef __JK_LEGACY_REALTIME_MOTION_BUFFER_H__
#define __JK_LEGACY_REALTIME_MOTION_BUFFER_H__

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "RealtimeMotionBuffer.hpp"

namespace RLController
{

    class LegacyRealtimeMotionBuffer
    {
    public:
        static constexpr int kFutureSteps = 5;

        LegacyRealtimeMotionBuffer(
            const std::vector<std::string> &joint_names,
            const std::string &zmq_endpoint,
            int zmq_hwm,
            int64_t dt_ns,
            int64_t tolerance_ns,
            const std::vector<int> &future_steps,
            const Eigen::VectorXf &default_joint_pos,
            const Eigen::Vector3f &default_root_pos,
            const Eigen::Quaternionf &default_root_quat,
            float root_height_offset = 0.0f);

        ~LegacyRealtimeMotionBuffer();

        LegacyRealtimeMotionBuffer(const LegacyRealtimeMotionBuffer &) = delete;
        LegacyRealtimeMotionBuffer &operator=(const LegacyRealtimeMotionBuffer &) = delete;

        bool ready() const;

        std::array<InterpolatedFrame, kFutureSteps> get_obs();

        int64_t latest_timestamp_ns() const;
        int num_joints() const { return _num_joints; }

    private:
        void zmq_thread_loop();
        void report_recv_frequency();
        void append_payload(const std::string &json_str);
        void fill_sample_frames(
            const std::array<int64_t, kFutureSteps> &target_times_ns,
            std::array<InterpolatedFrame, kFutureSteps> &out) const;
        void cleanup(int64_t cutoff_ns);

        int _num_joints;
        int64_t _dt_ns;
        int64_t _tolerance_ns;
        int64_t _delay_ns;
        int64_t _history_ns;
        int _min_future_step;
        int _max_future_step;
        std::vector<int> _future_steps;
        std::array<int64_t, kFutureSteps> _future_steps_ns;

        mutable std::mutex _lock;

        void *_zmq_context;
        void *_zmq_socket;
        std::thread _zmq_thread;
        std::atomic<bool> _running{false};
        std::string _zmq_endpoint;

        std::vector<int64_t> _timestamps_ns;
        std::vector<Eigen::Vector3f> _root_pos_frames;
        std::vector<Eigen::Quaternionf> _root_quat_frames;
        std::vector<Eigen::VectorXf> _joint_pos_frames;

        Eigen::VectorXf _default_joint_pos;
        Eigen::Vector3f _default_root_pos;
        Eigen::Quaternionf _default_root_quat;
        Eigen::Quaternionf _identity_quat;
        float _root_height_offset;

        // Receive frequency diagnostics
        uint64_t _recv_count = 0;
        uint64_t _recv_freq_last_count = 0;
        double _recv_freq_report_interval_s = 1.0;
        std::chrono::steady_clock::time_point _recv_freq_last_time;
        std::chrono::steady_clock::time_point _last_recv_time;
        bool _has_last_recv_time = false;
    };

} // namespace RLController

#endif
