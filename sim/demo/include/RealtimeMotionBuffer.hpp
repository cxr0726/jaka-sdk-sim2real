#ifndef __JK_REALTIME_MOTION_BUFFER_H__
#define __JK_REALTIME_MOTION_BUFFER_H__

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Dense>
#include "SourceTimeline.hpp"

namespace RLController
{

    struct InterpolatedFrame
    {
        Eigen::Vector3f root_pos_w;
        Eigen::Quaternionf root_quat_w;
        Eigen::VectorXf dof_pos;
    };

    struct MotionTimingConfig
    {
        // Source latches a declared field/clock per generation. Undeclared legacy
        // streams latch publish_t_ns. Publish/Arrival explicitly force their axes.
        // A missing selected field never causes a packet-by-packet fallback.
        enum class TimestampMode { Source, Publish, Arrival };
        TimestampMode timestamp_mode = TimestampMode::Source;
        SourceTimeline::Config clock;
        int64_t stale_timeout_ns = 500'000'000;
        int64_t reset_gap_ns = 500'000'000;
        int64_t discontinuity_ns = 250'000'000;
    };

    struct MotionWindow
    {
        enum class Status { Buffering, Tracking, Underrun, Stale };
        Status status = Status::Buffering;
        uint64_t generation = 0;
        std::array<InterpolatedFrame, 5> frames;
        int64_t query_steady_ns = 0;
        int64_t source_now_ns = 0;
        int64_t latest_recv_age_ns = -1;
        std::array<int64_t, 5> target_times_ns{};
        bool clamped_low = false;
        bool clamped_high = false;
        std::string source_time_field;
        std::string source_clock_id;
        bool reference_ready() const
        {
            return status == Status::Tracking || status == Status::Underrun;
        }
    };

    class RealtimeMotionBuffer
    {
    public:
        static constexpr int kFutureSteps = 5;

        RealtimeMotionBuffer(
            const std::vector<std::string> &joint_names,
            const std::string &zmq_endpoint,
            int zmq_hwm,
            int64_t dt_ns,
            int64_t tolerance_ns,
            const std::vector<int> &future_steps,
            const Eigen::VectorXf &default_joint_pos,
            const Eigen::Vector3f &default_root_pos,
            const Eigen::Quaternionf &default_root_quat,
            const MotionTimingConfig &timing = MotionTimingConfig{},
            int anchor_body_index = 0,
            float root_height_offset = 0.05f);

        ~RealtimeMotionBuffer();

        RealtimeMotionBuffer(const RealtimeMotionBuffer &) = delete;
        RealtimeMotionBuffer &operator=(const RealtimeMotionBuffer &) = delete;

        // Advisory result after sampling. Do not gate the first get_observation()
        // on this legacy helper; the snapshot is the authoritative status.
        bool ready() const;

        // One atomic clock/coverage/frame snapshot per control tick.
        MotionWindow get_observation();
        static const char *status_name(MotionWindow::Status status);

        // Legacy frames-only wrapper; it discards Buffering/Stale metadata.
        std::array<InterpolatedFrame, kFutureSteps> get_obs();

        int64_t latest_timestamp_ns() const; // SOURCE-RELATIVE nanoseconds, not Unix epoch
        int num_joints() const { return _num_joints; }

    private:
        void zmq_thread_loop();
        void report_recv_frequency();
        friend struct RealtimeMotionBufferTestAccess;
        void append_payload(const std::string &json_str, int64_t recv_ns);
        MotionWindow sample_locked(int64_t now_ns);
        void reset_stream_locked(const char *reason);
        void log_window_locked(const MotionWindow &window);
        void fill_sample_frames(
            const std::array<int64_t, kFutureSteps> &target_times_ns,
            std::array<InterpolatedFrame, kFutureSteps> &out) const;
        void cleanup(int64_t cutoff_ns);

        int _num_joints;
        int _anchor_body_index;
        float _root_height_offset;
        int64_t _dt_ns;
        int64_t _tolerance_ns;
        int64_t _delay_ns;
        int64_t _history_ns;
        int _min_future_step;
        int _max_future_step;
        std::vector<int> _future_steps;
        std::array<int64_t, kFutureSteps> _future_steps_ns;

        mutable std::mutex _lock;
        MotionTimingConfig _timing;
        SourceTimeline _timeline;
        uint64_t _generation = 0;
        int64_t _last_valid_recv_ns = -1;
        std::optional<int64_t> _last_source_ns;
        std::optional<int64_t> _last_seq;
        std::string _source_time_field;
        std::string _source_clock_id;
        std::optional<std::string> _stream_id;
        std::vector<std::string> _retired_stream_ids;
        bool _has_tracking_window = false;
        InterpolatedFrame _held_frame;
        int64_t _storage_ns = 0;
        std::ofstream _diagnostic_log;

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
