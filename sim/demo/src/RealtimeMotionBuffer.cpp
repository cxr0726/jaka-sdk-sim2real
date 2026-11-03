#include "RealtimeMotionBuffer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <stdexcept>

#include <zmq.h>
#include <zmq_addon.hpp>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace RLController
{

    namespace
    {
        int64_t steady_now_ns()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        int64_t nonnegative_integer(const json &value, const char *field)
        {
            if (!value.is_number_integer() ||
                (value.is_number_unsigned() && value.get<uint64_t>() > uint64_t(INT64_MAX)))
                throw std::runtime_error(std::string(field) + " must be an int64");
            const int64_t result = value.get<int64_t>();
            if (result < 0) throw std::runtime_error(std::string(field) + " must be nonnegative");
            return result;
        }

        Eigen::Quaternionf normalize_quat(const Eigen::Quaternionf &q)
        {
            if (q.norm() < 1.0e-6f)
            {
                return Eigen::Quaternionf::Identity();
            }
            Eigen::Quaternionf r = q;
            r.normalize();
            return r;
        }

        Eigen::Quaternionf slerp_quat(const Eigen::Quaternionf &q0_in,
                                      const Eigen::Quaternionf &q1_in,
                                      float t)
        {
            return normalize_quat(q0_in).slerp(t, normalize_quat(q1_in));
        }
    } // namespace

    RealtimeMotionBuffer::RealtimeMotionBuffer(
        const std::vector<std::string> &joint_names,
        const std::string &zmq_endpoint,
        int zmq_hwm,
        int64_t dt_ns,
        int64_t tolerance_ns,
        const std::vector<int> &future_steps,
        const Eigen::VectorXf &default_joint_pos,
        const Eigen::Vector3f &default_root_pos,
        const Eigen::Quaternionf &default_root_quat,
        const MotionTimingConfig &timing,
        int anchor_body_index,
        float root_height_offset)
        : _num_joints(static_cast<int>(joint_names.size())),
          _anchor_body_index(anchor_body_index),
          _root_height_offset(root_height_offset),
          _dt_ns(dt_ns),
          _tolerance_ns(tolerance_ns),
           _future_steps(future_steps),
           _timing(timing),
           _timeline(timing.clock),
           _zmq_context(nullptr),
           _zmq_socket(nullptr),
           _zmq_endpoint(zmq_endpoint),
           _default_joint_pos(default_joint_pos),
           _default_root_pos(default_root_pos),
           _default_root_quat(normalize_quat(default_root_quat)),
           _identity_quat(Eigen::Quaternionf::Identity()),
           _recv_freq_last_time(std::chrono::steady_clock::now())
    {
        if (dt_ns <= 0 || dt_ns > 1'000'000'000)
        {
            throw std::invalid_argument("dt_ns must be in (0, 1 second]");
        }
        if (static_cast<int>(_future_steps.size()) != kFutureSteps)
        {
            throw std::invalid_argument("future_steps must have exactly " +
                                        std::to_string(kFutureSteps) + " elements");
        }
        if (_num_joints <= 0)
        {
            throw std::invalid_argument("joint_names must not be empty");
        }
        if (_anchor_body_index < 0 || _anchor_body_index > 4095 ||
            !std::isfinite(_root_height_offset) || std::abs(_root_height_offset) > 1.f)
            throw std::invalid_argument("Invalid motion anchor index/height offset");
        if (tolerance_ns < 0 || tolerance_ns > 60'000'000'000LL ||
            timing.stale_timeout_ns <= 0 || timing.reset_gap_ns <= 0 ||
            timing.discontinuity_ns <= 0 || default_joint_pos.size() != _num_joints ||
            !default_joint_pos.allFinite() || !default_root_pos.allFinite() ||
            !default_root_quat.coeffs().allFinite() || !std::isfinite(default_root_quat.norm()) ||
            default_root_quat.norm() < 1.e-6f)
            throw std::invalid_argument("Invalid motion timing or default reference");
        for (int step : _future_steps)
            if (step < -10000 || step > 10000)
                throw std::invalid_argument("future_steps outside supported range [-10000, 10000]");
        for (size_t i = 1; i < _future_steps.size(); ++i)
            if (_future_steps[i] <= _future_steps[i - 1])
                throw std::invalid_argument("future_steps must be strictly increasing");

        _min_future_step = *std::min_element(_future_steps.begin(), _future_steps.end());
        _max_future_step = *std::max_element(_future_steps.begin(), _future_steps.end());

        for (int i = 0; i < kFutureSteps; ++i)
        {
            _future_steps_ns[i] = static_cast<int64_t>(_future_steps[i]) * _dt_ns;
        }

        _delay_ns = static_cast<int64_t>(_max_future_step) * _dt_ns + _tolerance_ns;
        _history_ns = _delay_ns + std::abs(_min_future_step) * _dt_ns;
        if (_delay_ns < 0) throw std::invalid_argument("Negative motion delay");
        _storage_ns = std::max(_history_ns + 2 * _dt_ns,
                              timing.clock.warmup_ns + _delay_ns + _dt_ns);
        _held_frame = {_default_root_pos, _default_root_quat, _default_joint_pos};

        if (const char *path = std::getenv("JAKA_MOTION_BUFFER_DIAG_CSV"); path && *path)
        {
            _diagnostic_log.open(path);
            if (!_diagnostic_log) throw std::runtime_error("Cannot open motion buffer diagnostic CSV");
            _diagnostic_log << "query_steady_ns,generation,status,source_now_ns,target0_ns,target4_ns,"
                               "tmin_ns,tmax_ns,n,clamped_low,clamped_high,latest_recv_age_ms,offset_ms,source_time_field,source_clock_id\n";
            _diagnostic_log << std::setprecision(12);
        }
        std::printf("[RealtimeMotionBuffer] clock=%s, warmup=%.0fms, stale=%.0fms, delay=%.0fms\n",
                    timing.timestamp_mode == MotionTimingConfig::TimestampMode::Source ? "declared-source-mapped-steady" :
                    timing.timestamp_mode == MotionTimingConfig::TimestampMode::Publish ? "publish-mapped-steady" : "arrival-steady",
                    timing.clock.warmup_ns / 1.e6, timing.stale_timeout_ns / 1.e6, _delay_ns / 1.e6);

        _running = true;

        if (!_zmq_endpoint.empty())
        {
            _zmq_context = zmq_ctx_new();
            if (!_zmq_context)
            {
                throw std::runtime_error("Failed to create ZMQ context");
            }

            _zmq_socket = zmq_socket(_zmq_context, ZMQ_SUB);
            if (!_zmq_socket)
            {
                zmq_ctx_destroy(_zmq_context);
                _zmq_context = nullptr;
                throw std::runtime_error("Failed to create ZMQ SUB socket");
            }

            int linger = 0;
            zmq_setsockopt(_zmq_socket, ZMQ_LINGER, &linger, sizeof(linger));
            zmq_setsockopt(_zmq_socket, ZMQ_RCVHWM, &zmq_hwm, sizeof(zmq_hwm));
            zmq_setsockopt(_zmq_socket, ZMQ_SUBSCRIBE, "", 0);

            int rc = zmq_connect(_zmq_socket, _zmq_endpoint.c_str());
            if (rc != 0)
            {
                std::fprintf(stderr, "[RealtimeMotionBuffer] ZMQ connect to %s failed: %s\n",
                             _zmq_endpoint.c_str(), zmq_strerror(zmq_errno()));
            }
            else
            {
                std::printf("[RealtimeMotionBuffer] ZMQ connected to %s\n", _zmq_endpoint.c_str());
            }

            _zmq_thread = std::thread(&RealtimeMotionBuffer::zmq_thread_loop, this);
        }
    }

    RealtimeMotionBuffer::~RealtimeMotionBuffer()
    {
        _running = false;
        if (_zmq_thread.joinable())
        {
            _zmq_thread.join();
        }
        if (_zmq_socket)
        {
            zmq_close(_zmq_socket);
            _zmq_socket = nullptr;
        }
        if (_zmq_context)
        {
            zmq_ctx_destroy(_zmq_context);
            _zmq_context = nullptr;
        }
    }

    bool RealtimeMotionBuffer::ready() const
    {
        std::lock_guard<std::mutex> lock(_lock);
        return _has_tracking_window && _timeline.locked() && _last_valid_recv_ns >= 0 &&
               steady_now_ns() - _last_valid_recv_ns < _timing.stale_timeout_ns;
    }

    void RealtimeMotionBuffer::report_recv_frequency()
    {
        const auto recv_now = std::chrono::steady_clock::now();
        const double recv_elapsed_s =
            std::chrono::duration<double>(recv_now - _recv_freq_last_time).count();
        if (recv_elapsed_s < _recv_freq_report_interval_s)
        {
            return;
        }

        const uint64_t delta = _recv_count - _recv_freq_last_count;
        const double recv_freq_hz = static_cast<double>(delta) / recv_elapsed_s;
        std::printf("[RealtimeMotionBuffer] recv freq: %.1f Hz (%llu pkts in %.3f s)\n",
                    recv_freq_hz, static_cast<unsigned long long>(delta), recv_elapsed_s);
        _recv_freq_last_count = _recv_count;
        _recv_freq_last_time = recv_now;
    }

    void RealtimeMotionBuffer::zmq_thread_loop()
    {
        while (_running)
        {
            report_recv_frequency();

            zmq_msg_t msg;
            zmq_msg_init(&msg);
            int rc = zmq_msg_recv(&msg, _zmq_socket, ZMQ_DONTWAIT);
            if (rc < 0)
            {
                zmq_msg_close(&msg);
                if (zmq_errno() == EAGAIN)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Stamp before copying or JSON parsing. This clock is receiver-local only.
            const int64_t recv_ns = steady_now_ns();
            std::string json_str;
            try
            {
                json_str.assign(static_cast<const char *>(zmq_msg_data(&msg)), zmq_msg_size(&msg));
            }
            catch (...)
            {
                zmq_msg_close(&msg);
                throw;
            }
            zmq_msg_close(&msg);
            try
            {
                append_payload(json_str, recv_ns);
            }
            catch (const std::exception &e)
            {
                std::fprintf(stderr, "[RealtimeMotionBuffer] append_payload error: %s\n", e.what());
            }
        }
    }

    void RealtimeMotionBuffer::append_payload(const std::string &json_str, int64_t recv_ns)
    {
        ++_recv_count;

        const auto recv_now = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(recv_ns));
        if (_has_last_recv_time)
        {
            const double gap_ms =
                std::chrono::duration<double, std::milli>(recv_now - _last_recv_time).count();
            const double normal_ms = static_cast<double>(_dt_ns) / 1.0e6;
            if (gap_ms > 2.0 * normal_ms)
            {
                std::printf("\n\033[1;31m"
                            "*** [RealtimeMotionBuffer] PACKET GAP: %.1f ms since last packet "
                            "(normal %.1f ms, > 2x) ***"
                            "\033[0m\n",
                            gap_ms, normal_ms);
            }
        }
        _last_recv_time = recv_now;
        _has_last_recv_time = true;

        json payload = json::parse(json_str);
        if (!payload.is_object())
        {
            throw std::runtime_error("Payload is not a JSON object");
        }

        std::optional<int64_t> seq;
        if (payload.contains("seq")) seq = nonnegative_integer(payload.at("seq"), "seq");
        std::optional<std::string> stream_id;
        if (payload.contains("stream_id"))
        {
            stream_id = payload.at("stream_id").get<std::string>();
            if (stream_id->empty() || stream_id->size() > 128)
                throw std::runtime_error("Invalid stream_id");
        }

        // Select a candidate protocol BEFORE comparing epochs with an old stream.
        // The candidate becomes active only after complete pose/session validation.
        std::string source_time_field = "publish_t_ns";
        std::string source_clock_id = "publish-wall";
        if (_timing.timestamp_mode == MotionTimingConfig::TimestampMode::Arrival)
        {
            source_time_field = "receiver_steady_ns";
            source_clock_id = "receiver-steady";
        }
        else if (_timing.timestamp_mode == MotionTimingConfig::TimestampMode::Source)
        {
            const bool has_field = payload.contains("source_time_field");
            const bool has_clock = payload.contains("source_clock_id");
            if (has_field != has_clock)
                throw std::runtime_error("Incomplete source clock declaration");
            if (has_field)
            {
                source_time_field = payload.at("source_time_field").get<std::string>();
                source_clock_id = payload.at("source_clock_id").get<std::string>();
                if (!stream_id ||
                    (source_time_field != "motion_t_ns" && source_time_field != "pico_recv_time_ns" && source_time_field != "publish_t_ns") ||
                    source_clock_id.empty() || source_clock_id.size() > 128 ||
                    source_clock_id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:._-/") != std::string::npos)
                    throw std::runtime_error("Invalid source clock declaration (stream_id required)");
            }
        }
        const int64_t source_ns = source_time_field == "receiver_steady_ns"
            ? recv_ns : nonnegative_integer(payload.at(source_time_field), source_time_field.c_str());
        if (source_ns <= 0 || recv_ns < 0) throw std::runtime_error("Invalid motion timestamp");

        json joint_pos_json;
        bool is_single_key = false;
        if (payload.contains("joint_pos"))
        {
            joint_pos_json = payload["joint_pos"];
            is_single_key = true;
        }
        else if (payload.contains("dof_pos"))
        {
            joint_pos_json = payload["dof_pos"];
        }
        else if (payload.contains("qpos"))
        {
            joint_pos_json = payload["qpos"];
        }
        else
        {
            throw std::runtime_error("Payload missing joint_pos/dof_pos/qpos");
        }

        std::vector<float> joint_pos_vec = joint_pos_json.get<std::vector<float>>();
        if (!is_single_key && static_cast<int>(joint_pos_vec.size()) >= 7 + _num_joints)
        {
            joint_pos_vec.erase(joint_pos_vec.begin(), joint_pos_vec.begin() + 7);
        }
        if (static_cast<int>(joint_pos_vec.size()) != _num_joints)
        {
            throw std::runtime_error("Expected " + std::to_string(_num_joints) +
                                     " joint positions, got " + std::to_string(joint_pos_vec.size()));
        }

        if (!payload.contains("body_pos_w") || !payload.contains("body_quat_w"))
        {
            throw std::runtime_error("Payload missing body_pos_w/body_quat_w");
        }

        auto body_pos_w = payload["body_pos_w"].get<std::vector<std::vector<float>>>();
        auto body_quat_w = payload["body_quat_w"].get<std::vector<std::vector<float>>>();

        if (body_pos_w.size() <= static_cast<size_t>(_anchor_body_index) ||
            body_quat_w.size() <= static_cast<size_t>(_anchor_body_index))
        {
            throw std::runtime_error("body_pos_w/body_quat_w missing selected anchor body");
        }

        // Incoming body order is JAKA: base_link=0, waist_yaw_Link=13.
        const auto &root_pos_vec = body_pos_w[_anchor_body_index];
        const auto &root_quat_vec = body_quat_w[_anchor_body_index];

        if (root_pos_vec.size() < 3 || root_quat_vec.size() < 4)
        {
            throw std::runtime_error("Root body pos/quat has wrong dimension");
        }

        Eigen::Vector3f root_pos(root_pos_vec[0], root_pos_vec[1],
                                root_pos_vec[2] + _root_height_offset);
        Eigen::Quaternionf root_quat(root_quat_vec[0], root_quat_vec[1],
                                     root_quat_vec[2], root_quat_vec[3]);
        if (!root_pos.allFinite() || !root_quat.coeffs().allFinite() ||
            !std::isfinite(root_quat.norm()) || root_quat.norm() < 1.e-6f ||
            !std::all_of(joint_pos_vec.begin(), joint_pos_vec.end(), [](float v) { return std::isfinite(v); }))
            throw std::runtime_error("Motion contains nonfinite values or zero quaternion");
        root_quat.normalize();

        Eigen::VectorXf joint_pos = Eigen::VectorXf::Map(joint_pos_vec.data(),
                                                         static_cast<Eigen::Index>(joint_pos_vec.size()));

        // Reorder from MuJoCo joint order to Isaac Lab joint order.
        static constexpr std::array<int, 27> kMujocoToIsaacIndex = {
            0, 6, 12, 1, 7, 13, 25, 19, 2, 8, 14, 26, 20,
            3, 9, 15, 21, 4, 10, 16, 22, 5, 11, 17, 23, 18, 24};
        if (static_cast<size_t>(joint_pos.size()) == kMujocoToIsaacIndex.size())
        {
            Eigen::VectorXf reordered(joint_pos.size());
            for (Eigen::Index m = 0; m < joint_pos.size(); ++m)
                reordered[m] = joint_pos[static_cast<Eigen::Index>(kMujocoToIsaacIndex[m])];
            joint_pos = reordered;
        }

        std::lock_guard<std::mutex> lock(_lock);
        // Validate the entire pose/protocol before mutating the clock or generation.
        const bool long_gap = _last_valid_recv_ns >= 0 && recv_ns - _last_valid_recv_ns >= _timing.reset_gap_ns;
        if (_stream_id && !stream_id && !long_gap) return; // never mix while a stream is active
        if (stream_id && std::find(_retired_stream_ids.begin(), _retired_stream_ids.end(), *stream_id) != _retired_stream_ids.end())
            return;
        const bool stream_change = stream_id != _stream_id && _last_valid_recv_ns >= 0;
        if (_last_valid_recv_ns >= 0 && recv_ns < _last_valid_recv_ns) return;
        if (_last_valid_recv_ns >= 0 && !stream_change && !long_gap &&
            (source_time_field != _source_time_field || source_clock_id != _source_clock_id))
            return; // same active stream must not switch fields or clock domains

        bool sequence_restart = false;
        if (!stream_change && !long_gap && seq && _last_seq && *seq <= *_last_seq)
        {
            // On an ordered single-PUB link, seq0 plus a newer source stamp is a
            // legacy publisher restart. Otherwise duplicates/older packets are dropped.
            sequence_restart = *seq == 0 && *seq < *_last_seq && _last_source_ns && source_ns > *_last_source_ns;
            if (!sequence_restart) return;
        }
        bool discontinuity = false;
        if (!stream_change && !long_gap && !sequence_restart && _last_source_ns)
        {
            if (source_ns == *_last_source_ns) return;
            if (source_ns < *_last_source_ns)
            {
                discontinuity = (*_last_source_ns - source_ns > _timing.discontinuity_ns) ||
                    (seq && _last_seq && *seq > *_last_seq);
                if (!discontinuity) return;
            }
            else
            {
                const int64_t source_delta = source_ns - *_last_source_ns;
                const int64_t recv_delta = recv_ns - _last_valid_recv_ns;
                discontinuity = std::abs(source_delta - recv_delta) > _timing.discontinuity_ns;
            }
        }
        const bool new_generation = _generation == 0 || stream_change || long_gap || sequence_restart || discontinuity;
        if (new_generation)
        {
            if (stream_change && _stream_id)
            {
                _retired_stream_ids.push_back(*_stream_id);
                if (_retired_stream_ids.size() > 8) _retired_stream_ids.erase(_retired_stream_ids.begin());
            }
            reset_stream_locked(_generation == 0 ? "initial" : stream_change ? "stream_id" :
                                long_gap ? "receive gap" : sequence_restart ? "seq restart" : "time discontinuity");
        }
        _stream_id = stream_id;
        const auto timestamp_ns = _timeline.observe(source_ns, recv_ns);
        if (!timestamp_ns) return;
        _source_time_field = source_time_field;
        _source_clock_id = source_clock_id;
        if (new_generation)
            std::printf("[RealtimeMotionBuffer] source_field=%s clock_id=%s\n",
                        _source_time_field.c_str(), _source_clock_id.c_str());
        _last_source_ns = source_ns;
        _last_seq = seq;
        _last_valid_recv_ns = recv_ns;
        _timestamps_ns.push_back(*timestamp_ns);
        _root_pos_frames.push_back(root_pos);
        _root_quat_frames.push_back(root_quat);
        _joint_pos_frames.push_back(joint_pos);
        // Cleanup also on reception: memory remains bounded if control stops querying.
        cleanup(*timestamp_ns - _storage_ns);
        if (_timestamps_ns.size() > 4096)
        {
            const auto excess = _timestamps_ns.size() - 4096;
            _timestamps_ns.erase(_timestamps_ns.begin(), _timestamps_ns.begin() + excess);
            _root_pos_frames.erase(_root_pos_frames.begin(), _root_pos_frames.begin() + excess);
            _root_quat_frames.erase(_root_quat_frames.begin(), _root_quat_frames.begin() + excess);
            _joint_pos_frames.erase(_joint_pos_frames.begin(), _joint_pos_frames.begin() + excess);
        }
    }

    void RealtimeMotionBuffer::reset_stream_locked(const char *reason)
    {
        _timeline.reset();
        _timestamps_ns.clear();
        _root_pos_frames.clear();
        _root_quat_frames.clear();
        _joint_pos_frames.clear();
        _last_source_ns.reset();
        _last_seq.reset();
        _source_time_field.clear();
        _source_clock_id.clear();
        _last_valid_recv_ns = -1;
        _has_tracking_window = false;
        ++_generation;
        std::printf("[RealtimeMotionBuffer] generation=%llu buffering (%s)\n",
                    static_cast<unsigned long long>(_generation), reason);
    }

    void RealtimeMotionBuffer::cleanup(int64_t cutoff_ns)
    {
        const auto after = std::lower_bound(_timestamps_ns.begin(), _timestamps_ns.end(), cutoff_ns);
        const auto preceding = std::distance(_timestamps_ns.begin(), after);
        // Keep one frame before the cutoff for interpolation, including at EOF.
        if (preceding > 1)
        {
            const auto count = preceding - 1;
            _timestamps_ns.erase(_timestamps_ns.begin(), _timestamps_ns.begin() + count);
            _root_pos_frames.erase(_root_pos_frames.begin(), _root_pos_frames.begin() + count);
            _root_quat_frames.erase(_root_quat_frames.begin(), _root_quat_frames.begin() + count);
            _joint_pos_frames.erase(_joint_pos_frames.begin(), _joint_pos_frames.begin() + count);
        }
    }

    void RealtimeMotionBuffer::fill_sample_frames(
        const std::array<int64_t, kFutureSteps> &target_times_ns,
        std::array<InterpolatedFrame, kFutureSteps> &out) const
    {
        if (_timestamps_ns.empty())
        {
            for (int i = 0; i < kFutureSteps; ++i)
            {
                out[i].root_pos_w = _default_root_pos;
                out[i].root_quat_w = _default_root_quat;
                out[i].dof_pos = _default_joint_pos;
            }
            return;
        }

        if (_timestamps_ns.size() == 1)
        {
            for (int i = 0; i < kFutureSteps; ++i)
            {
                out[i].root_pos_w = _root_pos_frames[0];
                out[i].root_quat_w = _root_quat_frames[0];
                out[i].dof_pos = _joint_pos_frames[0];
            }
            return;
        }

        const int64_t t_min = _timestamps_ns.front();
        const int64_t t_max = _timestamps_ns.back();
        const size_t n = _timestamps_ns.size();

        for (int step = 0; step < kFutureSteps; ++step)
        {
            int64_t t = target_times_ns[step];
            t = std::max(t_min, std::min(t_max, t));

            auto it = std::upper_bound(_timestamps_ns.begin(), _timestamps_ns.end(), t);
            size_t right_idx = static_cast<size_t>(std::distance(_timestamps_ns.begin(), it));
            if (right_idx >= n)
                right_idx = n - 1;
            if (right_idx == 0)
                right_idx = 1;
            size_t left_idx = right_idx - 1;

            int64_t t0 = _timestamps_ns[left_idx];
            int64_t t1 = _timestamps_ns[right_idx];
            float alpha = (t1 > t0) ? static_cast<float>(t - t0) / static_cast<float>(t1 - t0) : 0.0f;

            out[step].root_pos_w = _root_pos_frames[left_idx] +
                                   alpha * (_root_pos_frames[right_idx] - _root_pos_frames[left_idx]);
            out[step].root_quat_w = slerp_quat(_root_quat_frames[left_idx],
                                               _root_quat_frames[right_idx], alpha);
            out[step].dof_pos = _joint_pos_frames[left_idx] +
                                alpha * (_joint_pos_frames[right_idx] - _joint_pos_frames[left_idx]);
        }
    }

    const char *RealtimeMotionBuffer::status_name(MotionWindow::Status status)
    {
        switch (status)
        {
        case MotionWindow::Status::Buffering: return "buffering";
        case MotionWindow::Status::Tracking: return "tracking";
        case MotionWindow::Status::Underrun: return "underrun";
        case MotionWindow::Status::Stale: return "stale";
        }
        return "unknown";
    }

    MotionWindow RealtimeMotionBuffer::sample_locked(int64_t now_ns)
    {
        MotionWindow window;
        window.query_steady_ns = now_ns;
        window.generation = _generation;
        window.source_time_field = _source_time_field;
        window.source_clock_id = _source_clock_id;
        for (auto &frame : window.frames) frame = _held_frame;
        if (_last_valid_recv_ns >= 0)
            window.latest_recv_age_ns = std::max<int64_t>(0, now_ns - _last_valid_recv_ns);
        const auto source_now = _timeline.source_now(now_ns);
        if (source_now && !_timestamps_ns.empty())
        {
            window.source_now_ns = *source_now;
            for (int i = 0; i < kFutureSteps; ++i)
                window.target_times_ns[i] = *source_now - _delay_ns + _future_steps_ns[i];
            window.clamped_low = window.target_times_ns.front() < _timestamps_ns.front();
            window.clamped_high = window.target_times_ns.back() > _timestamps_ns.back();
            if (window.latest_recv_age_ns >= _timing.stale_timeout_ns)
            {
                window.status = MotionWindow::Status::Stale;
                // A single held reference, never a replay of the last varying preview.
            }
            else if (_has_tracking_window || (!window.clamped_low && !window.clamped_high))
            {
                fill_sample_frames(window.target_times_ns, window.frames);
                window.status = window.clamped_low || window.clamped_high
                    ? MotionWindow::Status::Underrun : MotionWindow::Status::Tracking;
                _has_tracking_window = true;
                _held_frame = window.frames[0];
            }
            cleanup(*source_now - _history_ns);
        }
        log_window_locked(window);
        return window;
    }

    void RealtimeMotionBuffer::log_window_locked(const MotionWindow &window)
    {
        if (!_diagnostic_log.is_open()) return;
        _diagnostic_log << window.query_steady_ns << ',' << window.generation << ',' << status_name(window.status)
            << ',' << window.source_now_ns << ',' << window.target_times_ns.front() << ',' << window.target_times_ns.back()
            << ',' << (_timestamps_ns.empty() ? -1 : _timestamps_ns.front())
            << ',' << (_timestamps_ns.empty() ? -1 : _timestamps_ns.back()) << ',' << _timestamps_ns.size()
            << ',' << window.clamped_low << ',' << window.clamped_high << ',' << window.latest_recv_age_ns / 1.e6
            << ',' << _timeline.offset_ns() / 1.e6
            << ',' << window.source_time_field << ',' << window.source_clock_id << '\n';
    }

    MotionWindow RealtimeMotionBuffer::get_observation()
    {
        std::lock_guard<std::mutex> lock(_lock);
        return sample_locked(steady_now_ns());
    }

    std::array<InterpolatedFrame, RealtimeMotionBuffer::kFutureSteps> RealtimeMotionBuffer::get_obs()
    {
        return get_observation().frames;
    }

    int64_t RealtimeMotionBuffer::latest_timestamp_ns() const
    {
        std::lock_guard<std::mutex> lock(_lock);
        if (_timestamps_ns.empty())
            return 0;
        return _timestamps_ns.back();
    }

} // namespace RLController
