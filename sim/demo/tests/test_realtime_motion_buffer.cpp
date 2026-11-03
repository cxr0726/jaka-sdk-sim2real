#include "RealtimeMotionBuffer.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace RLController
{
    // Inject clocks and payloads deterministically; no sockets, sleeps or viewer.
    struct RealtimeMotionBufferTestAccess
    {
        static void append(RealtimeMotionBuffer &buffer, const nlohmann::json &payload, int64_t recv_ns)
        {
            buffer.append_payload(payload.dump(), recv_ns);
        }
        static MotionWindow sample(RealtimeMotionBuffer &buffer, int64_t now_ns)
        {
            std::lock_guard<std::mutex> lock(buffer._lock);
            return buffer.sample_locked(now_ns);
        }
        struct Snapshot
        {
            uint64_t generation;
            int64_t last_recv;
            std::optional<int64_t> last_source;
            std::optional<int64_t> last_seq;
            std::vector<int64_t> keys;
            size_t clock_samples;
            int64_t storage_ns;
            std::string source_time_field;
            std::string source_clock_id;
        };
        static Snapshot snapshot(RealtimeMotionBuffer &buffer)
        {
            std::lock_guard<std::mutex> lock(buffer._lock);
            return {buffer._generation, buffer._last_valid_recv_ns, buffer._last_source_ns,
                    buffer._last_seq, buffer._timestamps_ns, buffer._timeline.sample_count(), buffer._storage_ns,
                    buffer._source_time_field, buffer._source_clock_id};
        }
    };
}

namespace
{
    using RLController::InterpolatedFrame;
    using RLController::MotionTimingConfig;
    using RLController::MotionWindow;
    using RLController::RealtimeMotionBuffer;
    using Access = RLController::RealtimeMotionBufferTestAccess;
    using json = nlohmann::json;
    constexpr int64_t ms = 1'000'000;
    constexpr int64_t source_epoch = 1'790'000'000'000'000'000LL;
    constexpr int64_t receiver_epoch = 120'000'000'000LL;
    constexpr int64_t sample_epoch = 5'000'000'000LL;

    void check(bool condition, const std::string &message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    void near(double value, double expected, const char *message, double tolerance = 2.e-6)
    {
        check(std::isfinite(value) && std::abs(value - expected) <= tolerance, message);
    }

    std::unique_ptr<RealtimeMotionBuffer> make_buffer(const MotionTimingConfig &timing = {}, int anchor_body_index = 0, float height_offset = .05f)
    {
        std::vector<std::string> joints;
        for (int i = 0; i < 27; ++i) joints.push_back("joint_" + std::to_string(i));
        return std::make_unique<RealtimeMotionBuffer>(
            joints, "", 1, 20 * ms, 40 * ms, std::vector<int>{0, 1, 2, 3, 4},
            Eigen::VectorXf::Zero(27), Eigen::Vector3f(0.f, 0.f, .8f),
            Eigen::Quaternionf::Identity(), timing, anchor_body_index, height_offset);
    }

    json packet(int64_t seq, double pose_time_s, int64_t source_ns, const std::string &stream_id = "")
    {
        std::vector<float> joints(27);
        for (int j = 0; j < 27; ++j) joints[j] = static_cast<float>(pose_time_s + .01 * j);
        std::vector<std::vector<float>> positions(14, {0.f, 0.f, 0.f});
        positions[0] = {static_cast<float>(pose_time_s), 0.f, .7f};
        std::vector<std::vector<float>> quaternions(14, {1.f, 0.f, 0.f, 0.f});
        quaternions[0] = {static_cast<float>(std::cos(.15 * pose_time_s)), 0.f, 0.f,
                           static_cast<float>(std::sin(.15 * pose_time_s))};
        json result = {{"seq", seq}, {"publish_t_ns", source_ns}, {"paused", false},
                       {"joint_pos", joints}, {"body_pos_w", positions}, {"body_quat_w", quaternions}};
        if (!stream_id.empty()) result["stream_id"] = stream_id;
        return result;
    }

    json declared_packet(int64_t seq, double pose_time_s, int64_t sample_ns, int64_t publish_ns,
                         const std::string &stream_id = "sample-publisher-A",
                         const std::string &clock_id = "python.monotonic:sample-publisher-A",
                         const std::string &field = "motion_t_ns")
    {
        auto result = packet(seq, pose_time_s, publish_ns, stream_id);
        result["source_time_field"] = field;
        result["source_clock_id"] = clock_id;
        result[field] = sample_ns;
        return result;
    }

    void seed_declared(RealtimeMotionBuffer &buffer)
    {
        for (int i = 0; i <= 30; ++i)
        {
            // Publication wall time repeatedly jumps by ten seconds. The chosen
            // pre-GMR monotonic sampling clock remains a regular 20ms trajectory.
            const int64_t publish_ns = source_epoch + i * 20 * ms + (i % 2 ? 10'000 * ms : 0);
            Access::append(buffer, declared_packet(i, .02 * i, sample_epoch + i * 20 * ms, publish_ns),
                           receiver_epoch + i * 20 * ms);
        }
    }

    void seed(RealtimeMotionBuffer &buffer, const std::string &stream_id = "", int64_t offset = 0)
    {
        for (int i = 0; i <= 30; ++i)
            Access::append(buffer, packet(i, .02 * i, source_epoch + offset + i * 20 * ms, stream_id),
                           receiver_epoch + i * 20 * ms);
    }

    void same_state(const Access::Snapshot &before, const Access::Snapshot &after, const char *message)
    {
        check(before.generation == after.generation && before.last_recv == after.last_recv &&
                  before.last_source == after.last_source && before.last_seq == after.last_seq &&
                  before.keys == after.keys && before.clock_samples == after.clock_samples &&
                  before.source_time_field == after.source_time_field && before.source_clock_id == after.source_clock_id,
              message);
    }

    void same_frame(const InterpolatedFrame &a, const InterpolatedFrame &b, const char *message)
    {
        check((a.root_pos_w - b.root_pos_w).norm() < 2.e-6f &&
                  a.root_quat_w.angularDistance(b.root_quat_w) < 2.e-6f &&
                  a.dof_pos.size() == b.dof_pos.size() && (a.dof_pos - b.dof_pos).norm() < 2.e-6f,
              message);
    }

    void test_warmup_and_interpolation()
    {
        auto buffer = make_buffer();
        auto empty = Access::sample(*buffer, receiver_epoch);
        check(empty.status == MotionWindow::Status::Buffering && !empty.reference_ready(), "empty buffer is ready");
        for (const auto &frame : empty.frames)
        {
            near(frame.root_pos_w.z(), .8, "buffering should retain configured reference");
            check(frame.dof_pos.size() == 27 && frame.dof_pos.allFinite(), "uninitialized buffering frame");
        }
        for (int i = 0; i < 25; ++i)
            Access::append(*buffer, packet(i, .02 * i, source_epoch + i * 20 * ms), receiver_epoch + i * 20 * ms);
        check(Access::sample(*buffer, receiver_epoch + 480 * ms).status == MotionWindow::Status::Buffering,
              "buffer became ready before clock warmup");
        for (int i = 25; i <= 30; ++i)
            Access::append(*buffer, packet(i, .02 * i, source_epoch + i * 20 * ms), receiver_epoch + i * 20 * ms);

        auto window = Access::sample(*buffer, receiver_epoch + 610 * ms);
        check(window.status == MotionWindow::Status::Tracking && window.reference_ready(), "complete window not tracking");
        check(!window.clamped_low && !window.clamped_high, "normal window unexpectedly clamped");
        check(window.target_times_ns[4] - window.target_times_ns[0] == 80 * ms, "preview is not 80ms");
        near(window.latest_recv_age_ns, 10 * ms, "receive age clock domain wrong", .1);
        // The desired times are 490..570ms, halfway between recorded packets.
        for (int i = 0; i < 5; ++i)
        {
            const double t = .49 + .02 * i;
            near(window.frames[i].root_pos_w.x(), t, "root linear interpolation wrong");
            near(window.frames[i].root_pos_w.z(), .75, "intentional 5cm height changed");
            near(window.frames[i].dof_pos[0], t, "joint interpolation wrong");
            near(window.frames[i].dof_pos[1], t + .06, "JAKA to Isaac reorder changed");
            const Eigen::Quaternionf expected(Eigen::AngleAxisf(static_cast<float>(.3 * t), Eigen::Vector3f::UnitZ()));
            near(window.frames[i].root_quat_w.angularDistance(expected), 0, "quaternion SLERP wrong", 2.e-6);
        }
        check(buffer->latest_timestamp_ns() == 600 * ms, "latest timestamp is not source-relative");
    }

    void test_constant_offsets()
    {
        auto baseline = make_buffer();
        seed(*baseline);
        auto expected = Access::sample(*baseline, receiver_epoch + 610 * ms);
        for (int64_t offset : std::array<int64_t, 4>{-150 * ms, 250 * ms, -86'400'000 * ms, 86'400'000 * ms})
        {
            auto shifted = make_buffer();
            seed(*shifted, "", offset);
            auto actual = Access::sample(*shifted, receiver_epoch + 610 * ms);
            check(actual.status == expected.status && actual.target_times_ns == expected.target_times_ns,
                  "remote epoch offset changes target times/status");
            for (int i = 0; i < 5; ++i) same_frame(actual.frames[i], expected.frames[i], "remote offset changes observation");
        }
    }

    void test_selected_anchor_and_height_offset()
    {
        auto base = make_buffer({}, 0, .05f);
        auto waist = make_buffer({}, 13, 0.f);
        for (int i = 0; i <= 30; ++i)
        {
            auto payload = packet(i, .02 * i, source_epoch + i * 20 * ms);
            payload["body_pos_w"][13] = std::vector<float>{9.f, 8.f, 7.f};
            Access::append(*base, payload, receiver_epoch + i * 20 * ms);
            Access::append(*waist, payload, receiver_epoch + i * 20 * ms);
        }
        const auto a = Access::sample(*base, receiver_epoch + 610 * ms);
        const auto b = Access::sample(*waist, receiver_epoch + 610 * ms);
        near(a.frames[0].root_pos_w[2], .75, "configured base height offset missing");
        near(b.frames[0].root_pos_w[0], 9., "selected waist index ignored");
        near(b.frames[0].root_pos_w[2], 7., "disabled height offset still applied");
        auto one_body = packet(31, .62, source_epoch + 620 * ms);
        one_body["body_pos_w"].erase(one_body["body_pos_w"].begin() + 1, one_body["body_pos_w"].end());
        one_body["body_quat_w"].erase(one_body["body_quat_w"].begin() + 1, one_body["body_quat_w"].end());
        Access::append(*base, one_body, receiver_epoch + 620 * ms);
        check(base->latest_timestamp_ns() == 620 * ms, "base-only body packet was rejected");
    }

    void test_invalid_payload_does_not_mutate_timeline()
    {
        auto buffer = make_buffer();
        seed(*buffer);
        const auto before = Access::snapshot(*buffer);
        auto valid = packet(31, .62, source_epoch + 620 * ms);
        std::vector<json> invalid;
        auto j = valid; j["body_pos_w"] = std::vector<std::vector<float>>(0, {0.f, 0.f, 0.f}); invalid.push_back(j);
        j = valid; j["body_quat_w"] = std::vector<std::vector<float>>(0, {1.f, 0.f, 0.f, 0.f}); invalid.push_back(j);
        j = valid; j["joint_pos"][0] = 1.e100; invalid.push_back(j); // finite JSON double, overflows a float
        j = valid; j["body_pos_w"][0][2] = 1.e100; invalid.push_back(j);
        j = valid; j["body_quat_w"][0] = std::vector<float>{0, 0, 0, 0}; invalid.push_back(j);
        j = valid; j["joint_pos"] = std::vector<float>(26, 0); invalid.push_back(j);
        j = valid; j.erase("publish_t_ns"); j["smplx_t_ns"] = source_epoch; invalid.push_back(j);
        j = valid; j["publish_t_ns"] = static_cast<double>(source_epoch); invalid.push_back(j);
        j = valid; j["publish_t_ns"] = uint64_t(std::numeric_limits<int64_t>::max()) + 1; invalid.push_back(j);
        j = valid; j["publish_t_ns"] = -1; invalid.push_back(j);
        for (const auto &bad : invalid)
        {
            try { Access::append(*buffer, bad, receiver_epoch + 620 * ms); }
            catch (const std::exception &) {}
            same_state(before, Access::snapshot(*buffer), "malformed payload mutated accepted timeline state");
        }
        Access::append(*buffer, valid, receiver_epoch + 620 * ms);
        check(buffer->latest_timestamp_ns() == 620 * ms, "invalid payload poisoned next valid packet");
    }

    void test_duplicates_and_sequence_restart()
    {
        auto buffer = make_buffer();
        seed(*buffer);
        const auto before = Access::snapshot(*buffer);
        Access::append(*buffer, packet(30, .62, source_epoch + 620 * ms), receiver_epoch + 620 * ms);
        Access::append(*buffer, packet(29, .64, source_epoch + 640 * ms), receiver_epoch + 640 * ms);
        Access::append(*buffer, packet(31, .6, source_epoch + 600 * ms), receiver_epoch + 650 * ms);
        same_state(before, Access::snapshot(*buffer), "duplicate/older packet changed active generation");
        Access::append(*buffer, packet(31, .62, source_epoch + 620 * ms), receiver_epoch + 660 * ms);
        check(Access::snapshot(*buffer).generation == before.generation, "valid continuation caused reset");
        Access::append(*buffer, packet(0, .64, source_epoch + 640 * ms), receiver_epoch + 680 * ms);
        const auto restart = Access::snapshot(*buffer);
        check(restart.generation == before.generation + 1 && restart.keys.size() == 1 && restart.keys[0] == 0,
              "legacy seq0 restart did not create fresh relative timeline");
        check(Access::sample(*buffer, receiver_epoch + 680 * ms).status == MotionWindow::Status::Buffering,
              "restart did not require rebuffering");
    }

    void test_stream_identity_and_clock_discontinuities()
    {
        auto buffer = make_buffer();
        seed(*buffer, "publisher-A");
        const auto before = Access::snapshot(*buffer);
        Access::append(*buffer, packet(0, .62, source_epoch + 620 * ms, "publisher-B"), receiver_epoch + 620 * ms);
        auto changed = Access::snapshot(*buffer);
        check(changed.generation == before.generation + 1 && changed.keys.size() == 1,
              "stream identity change did not reset generation");
        Access::append(*buffer, packet(31, .64, source_epoch + 640 * ms, "publisher-A"), receiver_epoch + 640 * ms);
        same_state(changed, Access::snapshot(*buffer), "retired stream reentered active timeline");
        Access::append(*buffer, packet(1, .64, source_epoch + 640 * ms), receiver_epoch + 640 * ms);
        same_state(changed, Access::snapshot(*buffer), "anonymous frame mixed into identified stream");
        Access::append(*buffer, packet(0, 1.2, source_epoch + 1200 * ms), receiver_epoch + 1200 * ms);
        const auto anonymous = Access::snapshot(*buffer);
        check(anonymous.generation == changed.generation + 1 && anonymous.keys.size() == 1,
              "legacy anonymous publisher cannot recover after identified stream ends");

        for (bool backwards : {false, true})
        {
            auto clock_jump = make_buffer();
            seed(*clock_jump);
            const auto current = Access::snapshot(*clock_jump);
            const int64_t next_source = source_epoch + (backwards ? 100 : 2000) * ms;
            Access::append(*clock_jump, packet(31, .62, next_source), receiver_epoch + 620 * ms);
            const auto reset = Access::snapshot(*clock_jump);
            check(reset.generation == current.generation + 1 && reset.keys.size() == 1 && reset.keys[0] == 0,
                  "forward/backward source-clock step was not rebuffered");
        }
    }

    void test_underrun_stale_and_long_gap_recovery()
    {
        auto buffer = make_buffer();
        seed(*buffer);
        const auto tracking = Access::sample(*buffer, receiver_epoch + 610 * ms);
        auto transient = Access::sample(*buffer, receiver_epoch + 670 * ms);
        check(transient.status == MotionWindow::Status::Underrun && transient.clamped_high && transient.reference_ready(),
              "transient uncovered tail not identified");
        near(transient.frames.front().root_pos_w.x(), .55, "underrun discarded covered part of window");
        near(transient.frames.back().root_pos_w.x(), .6, "underrun did not clamp missing tail");
        auto stale = Access::sample(*buffer, receiver_epoch + 1100 * ms);
        check(stale.status == MotionWindow::Status::Stale && !stale.reference_ready(), "500ms old input not stale");
        for (const auto &frame : stale.frames)
            same_frame(frame, transient.frames.front(), "stale state replayed old varying preview");

        const auto generation = stale.generation;
        // Exactly the configured gap threshold must create a new generation.
        Access::append(*buffer, packet(31, 1.1, source_epoch + 1100 * ms), receiver_epoch + 1100 * ms);
        auto buffering = Access::sample(*buffer, receiver_epoch + 1100 * ms);
        check(buffering.generation == generation + 1 && buffering.status == MotionWindow::Status::Buffering,
              "long-gap continuation did not rebuffer");
        for (const auto &frame : buffering.frames)
            same_frame(frame, transient.frames.front(), "rebuffering lost last consumed hold reference");
        for (int i = 1; i <= 25; ++i)
            Access::append(*buffer, packet(31 + i, 1.1 + .02 * i, source_epoch + (1100 + i * 20) * ms),
                           receiver_epoch + (1100 + i * 20) * ms);
        const auto recovered = Access::sample(*buffer, receiver_epoch + 1600 * ms);
        check(recovered.status == MotionWindow::Status::Tracking && recovered.generation == generation + 1,
              "fresh generation failed to relock");
        check(recovered.frames[0].root_pos_w.x() > tracking.frames[0].root_pos_w.x(), "recovery still using old stream");
    }

    void test_intentional_pause_and_fixed_arrival_mode()
    {
        auto buffer = make_buffer();
        for (int i = 0; i <= 100; ++i)
        {
            auto paused = packet(i, .2, source_epoch + i * 20 * ms);
            paused["paused"] = true;
            Access::append(*buffer, paused, receiver_epoch + i * 20 * ms);
            if (i >= 25)
            {
                auto window = Access::sample(*buffer, receiver_epoch + i * 20 * ms);
                check(window.status == MotionWindow::Status::Tracking && window.generation == 1,
                      "intentional static pause treated as source-clock failure");
                for (const auto &frame : window.frames)
                    near(frame.root_pos_w.x(), .2, "paused reference changed");
            }
        }

        MotionTimingConfig timing;
        timing.timestamp_mode = MotionTimingConfig::TimestampMode::Arrival;
        auto arrival = make_buffer(timing);
        for (int i = 0; i <= 30; ++i)
        {
            auto j = packet(i, .02 * i, source_epoch + i * 20 * ms);
            j.erase("publish_t_ns");
            j["smplx_t_ns"] = source_epoch - 5'000 * ms; // ignored in fixed arrival mode
            Access::append(*arrival, j, receiver_epoch + i * 20 * ms);
        }
        auto window = Access::sample(*arrival, receiver_epoch + 610 * ms);
        check(window.status == MotionWindow::Status::Tracking, "explicit arrival mode requires source field");
        near(window.frames[0].root_pos_w.x(), .49, "arrival mode uses another clock field");
    }

    void test_receive_side_storage_bound()
    {
        auto buffer = make_buffer();
        for (int i = 0; i <= 1200; ++i)
            Access::append(*buffer, packet(i, .02 * i, source_epoch + i * 20 * ms), receiver_epoch + i * 20 * ms);
        const auto state = Access::snapshot(*buffer);
        check(state.keys.size() > 1 && state.keys.size() <= static_cast<size_t>(state.storage_ns / (20 * ms) + 2),
              "receive-only stream grows without bound");
        check(state.keys.back() - state.keys.front() <= state.storage_ns + 20 * ms,
              "receive cleanup retained excessive source history");
        for (size_t i = 1; i < state.keys.size(); ++i)
            check(state.keys[i] > state.keys[i - 1], "source keys not immutable/increasing");
        check(Access::sample(*buffer, receiver_epoch + 24'000 * ms).status == MotionWindow::Status::Tracking,
              "bounded storage cannot cover current preview");
    }

    void test_declared_sampling_axis_and_legacy_selection()
    {
        auto baseline = make_buffer();
        seed(*baseline);
        const auto expected = Access::sample(*baseline, receiver_epoch + 610 * ms);

        auto declared = make_buffer();
        seed_declared(*declared);
        const auto state = Access::snapshot(*declared);
        check(state.generation == 1 && state.last_source == sample_epoch + 600 * ms,
              "declared monotonic source clock replaced by publication wall time");
        check(state.source_time_field == "motion_t_ns" &&
                  state.source_clock_id == "python.monotonic:sample-publisher-A",
              "selected source contract was not latched");
        for (size_t i = 1; i < state.keys.size(); ++i)
            check(state.keys[i] - state.keys[i - 1] == 20 * ms,
                  "variable post-GMR publication times warped source sample keys");
        const auto actual = Access::sample(*declared, receiver_epoch + 610 * ms);
        check(actual.status == MotionWindow::Status::Tracking && actual.target_times_ns == expected.target_times_ns,
              "declared sample clock did not preserve the reference window");
        for (int i = 0; i < 5; ++i) same_frame(actual.frames[i], expected.frames[i], "declared-clock interpolation changed");
        for (int i = 31; i <= 130; ++i)
        {
            auto paused = declared_packet(i, .6, sample_epoch + i * 20 * ms,
                                          source_epoch + i * 20 * ms + (i % 2 ? 10'000 * ms : 0));
            paused["paused"] = true;
            Access::append(*declared, paused, receiver_epoch + i * 20 * ms);
            const auto hold = Access::sample(*declared, receiver_epoch + i * 20 * ms);
            check(hold.status == MotionWindow::Status::Tracking && hold.generation == 1,
                  "fresh declared static-pause heartbeats reset the source clock");
            if (i >= 37)
                for (const auto &frame : hold.frames) near(frame.root_pos_w.x(), .6, "paused reference did not hold");
        }

        auto legacy = make_buffer();
        for (int i = 0; i <= 30; ++i)
        {
            auto value = packet(i, .02 * i, source_epoch + i * 20 * ms);
            value["motion_t_ns"] = -1; // Extra undeclared fields are not clock negotiation.
            value["pico_recv_time_ns"] = sample_epoch + i * 47 * ms;
            Access::append(*legacy, value, receiver_epoch + i * 20 * ms);
        }
        const auto legacy_state = Access::snapshot(*legacy);
        check(legacy_state.source_time_field == "publish_t_ns" && legacy_state.generation == 1,
              "legacy publisher silently selected an undeclared clock field");
        const auto legacy_window = Access::sample(*legacy, receiver_epoch + 610 * ms);
        for (int i = 0; i < 5; ++i) same_frame(legacy_window.frames[i], expected.frames[i], "legacy clock fallback changed");
    }

    void test_declared_contract_rejection_is_transactional()
    {
        auto buffer = make_buffer();
        seed_declared(*buffer);
        const auto tracking = Access::sample(*buffer, receiver_epoch + 610 * ms);
        const auto before = Access::snapshot(*buffer);
        const auto valid = declared_packet(31, .62, sample_epoch + 620 * ms, source_epoch + 620 * ms);
        std::vector<json> invalid;
        auto value = valid; value.erase("motion_t_ns"); invalid.push_back(value);
        value = valid; value.erase("source_time_field"); invalid.push_back(value);
        value = valid; value.erase("source_clock_id"); invalid.push_back(value);
        value = valid; value.erase("source_time_field"); value.erase("source_clock_id"); invalid.push_back(value);
        value = valid; value["motion_t_ns"] = static_cast<double>(sample_epoch + 620 * ms); invalid.push_back(value);
        value = valid; value["source_time_field"] = "smplx_t_ns"; value["smplx_t_ns"] = sample_epoch + 620 * ms; invalid.push_back(value);
        value = valid; value["source_clock_id"] = "other-clock"; invalid.push_back(value);
        value = valid; value["source_clock_id"] = "bad,clock"; invalid.push_back(value);
        value = valid; value["source_time_field"] = "pico_recv_time_ns"; value["pico_recv_time_ns"] = source_epoch + 620 * ms; invalid.push_back(value);
        value = valid; value["stream_id"] = "new-publisher-with-invalid-pose";
        value["source_clock_id"] = "python.monotonic:new-publisher";
        value["body_pos_w"] = std::vector<std::vector<float>>(0, {0.f, 0.f, 0.f}); invalid.push_back(value);
        for (const auto &bad : invalid)
        {
            try { Access::append(*buffer, bad, receiver_epoch + 620 * ms); }
            catch (const std::exception &) {}
            same_state(before, Access::snapshot(*buffer), "invalid/changed contract mutated active timeline");
        }
        const auto stale = Access::sample(*buffer, receiver_epoch + 1100 * ms);
        check(stale.status == MotionWindow::Status::Stale && stale.latest_recv_age_ns == 500 * ms,
              "rejected contract refreshed effective receive age");
        for (const auto &frame : stale.frames) same_frame(frame, tracking.frames.front(), "rejection changed held reference");

        auto empty = make_buffer();
        const auto empty_state = Access::snapshot(*empty);
        value = valid; value.erase("stream_id");
        try { Access::append(*empty, value, receiver_epoch); }
        catch (const std::exception &) {}
        same_state(empty_state, Access::snapshot(*empty), "unidentified declaration established a generation");
    }

    void test_source_contract_changes_on_stream_or_long_gap()
    {
        auto buffer = make_buffer();
        seed(*buffer, "legacy-publisher-A");
        const auto previous = Access::sample(*buffer, receiver_epoch + 610 * ms);
        const auto before = Access::snapshot(*buffer);
        // New stream selects a radically different clock origin. It must be
        // accepted before comparing against the old Unix source clock.
        Access::append(*buffer, declared_packet(0, .62, sample_epoch, source_epoch + 620 * ms,
                                               "sample-publisher-B", "python.monotonic:sample-publisher-B"),
                       receiver_epoch + 620 * ms);
        auto changed = Access::snapshot(*buffer);
        check(changed.generation == before.generation + 1 && changed.keys.size() == 1 && changed.keys.front() == 0 &&
                  changed.last_source == sample_epoch && changed.source_time_field == "motion_t_ns",
              "new stream clock origin/field rejected by old source comparison");
        auto buffering = Access::sample(*buffer, receiver_epoch + 620 * ms);
        check(buffering.status == MotionWindow::Status::Buffering, "clock selection skipped generation precharge");
        for (const auto &frame : buffering.frames) same_frame(frame, previous.frames.front(), "new clock lost hold reference");
        for (int i = 1; i <= 25; ++i)
            Access::append(*buffer, declared_packet(i, .62 + .02 * i, sample_epoch + i * 20 * ms,
                                                   source_epoch + (620 + i * 20) * ms,
                                                   "sample-publisher-B", "python.monotonic:sample-publisher-B"),
                           receiver_epoch + (620 + i * 20) * ms);
        const auto recovered = Access::sample(*buffer, receiver_epoch + 1130 * ms);
        check(recovered.status == MotionWindow::Status::Tracking && recovered.generation == changed.generation,
              "new declared source failed to rewarm");
        near(recovered.frames.front().root_pos_w.x(), 1.01, "new clock trajectory uses old epoch");

        Access::append(*buffer, declared_packet(26, 1.8, source_epoch + 1800 * ms, source_epoch + 1820 * ms,
                                               "sample-publisher-B", "publisher-wall:new-domain", "pico_recv_time_ns"),
                       receiver_epoch + 1800 * ms);
        const auto after_gap = Access::snapshot(*buffer);
        check(after_gap.generation == changed.generation + 1 && after_gap.keys.size() == 1 &&
                  after_gap.source_time_field == "pico_recv_time_ns" && after_gap.source_clock_id == "publisher-wall:new-domain",
              "long-gap clock contract change not latched as a new generation");
    }

    void test_explicit_modes_ignore_source_declaration()
    {
        auto baseline = make_buffer();
        seed(*baseline);
        const auto expected = Access::sample(*baseline, receiver_epoch + 610 * ms);
        for (const auto mode : {MotionTimingConfig::TimestampMode::Publish, MotionTimingConfig::TimestampMode::Arrival})
        {
            MotionTimingConfig timing;
            timing.timestamp_mode = mode;
            auto fixed = make_buffer(timing);
            for (int i = 0; i <= 30; ++i)
            {
                auto value = declared_packet(i, .02 * i, sample_epoch + i * 40 * ms, source_epoch + i * 20 * ms);
                if (mode == MotionTimingConfig::TimestampMode::Arrival)
                    value["publish_t_ns"] = source_epoch + i * 20 * ms + (i % 2 ? 10'000 * ms : 0);
                Access::append(*fixed, value, receiver_epoch + i * 20 * ms);
            }
            const auto actual = Access::sample(*fixed, receiver_epoch + 610 * ms);
            check(actual.status == MotionWindow::Status::Tracking && actual.generation == 1,
                  "explicit fixed clock mode follows packet clock declaration");
            for (int i = 0; i < 5; ++i) same_frame(actual.frames[i], expected.frames[i], "explicit mode changed interpolation axis");
        }
    }

    void test_pre_gmr_pause_resume_availability_budget()
    {
        // Keep honest pre-GMR source times across pause/live. Pause has no GMR
        // cost; live takes 20ms, followed by 1ms network delivery in both modes.
        // A 10s low quantile still includes the faster paused path. The source
        // clock is continuous, but a 40ms tail need not cover a 30Hz live stream.
        std::vector<std::string> joints(27, "joint");
        const auto make_with_tolerance = [&](int64_t tolerance) {
            return std::make_unique<RealtimeMotionBuffer>(
                joints, "", 1, 20 * ms, tolerance, std::vector<int>{0, 1, 2, 3, 4},
                Eigen::VectorXf::Zero(27), Eigen::Vector3f(0.f, 0.f, .8f), Eigen::Quaternionf::Identity());
        };
        auto tail40 = make_with_tolerance(40 * ms);
        auto tail60 = make_with_tolerance(60 * ms);
        constexpr int64_t source_period_ns = 33'333'333; // 30Hz, separate from the 50Hz control queries
        int next_packet = 0;
        int live_queries = 0, underruns40 = 0, tracking40 = 0, underruns60 = 0, tracking60 = 0;
        for (int query = 0; query <= 150; ++query)
        {
            const int64_t now_rel = query * 20 * ms;
            while (true)
            {
                const int64_t sample_rel = next_packet * source_period_ns;
                const bool paused = sample_rel < 1000 * ms;
                const int64_t gmr_cost = paused ? 0 : 20 * ms;
                const int64_t delivery_rel = sample_rel + gmr_cost + ms;
                if (delivery_rel > now_rel) break; // never expose an unreceived reference to get_obs
                const double pose_time_s = paused ? .2 : .2 + .1 * (sample_rel / 1.e9 - 1.);
                auto value = declared_packet(next_packet, pose_time_s, sample_epoch + sample_rel,
                                             source_epoch + sample_rel + gmr_cost);
                value["paused"] = paused;
                Access::append(*tail40, value, receiver_epoch + delivery_rel);
                Access::append(*tail60, value, receiver_epoch + delivery_rel);
                ++next_packet;
            }
            const auto window40 = Access::sample(*tail40, receiver_epoch + now_rel);
            const auto window60 = Access::sample(*tail60, receiver_epoch + now_rel);
            if (now_rel < 1200 * ms) continue;
            ++live_queries;
            check(window40.generation == 1 && window60.generation == 1,
                  "valid pause-to-live availability change reset the source clock");
            check(window40.status == MotionWindow::Status::Tracking || window40.status == MotionWindow::Status::Underrun,
                  "healthy live source became buffering/stale");
            check(window60.status == MotionWindow::Status::Tracking || window60.status == MotionWindow::Status::Underrun,
                  "larger tolerance changed stream liveness");
            if (window40.status == MotionWindow::Status::Underrun)
            {
                ++underruns40;
                check(window40.clamped_high && !window40.clamped_low,
                      "40ms tail failed for a reason other than late live availability");
            }
            else ++tracking40;
            if (window60.status == MotionWindow::Status::Underrun) ++underruns60;
            else
            {
                ++tracking60;
                check(!window60.clamped_low && !window60.clamped_high,
                      "60ms tolerance claimed Tracking with missing samples");
            }
            check(window60.target_times_ns[0] == window40.target_times_ns[0] - 20 * ms &&
                      window60.target_times_ns[4] == window40.target_times_ns[4] - 20 * ms,
                  "tolerance did not shift every target by the same amount");
            check(window40.target_times_ns[4] - window40.target_times_ns[0] == 80 * ms &&
                      window60.target_times_ns[4] - window60.target_times_ns[0] == 80 * ms,
                  "tolerance changed the policy preview span");
        }
        check(live_queries == 91 && underruns40 > 0 && tracking40 > 0,
              "availability fixture did not expose a finite intermittent tail deficit");
        check(underruns60 == 0 && tracking60 == live_queries,
              "60ms tail failed to cover the fixture's bounded delivery delay");
        std::cout << "Pre-GMR availability fixture: " << live_queries
                  << " live queries, tolerance40 Tracking=" << tracking40 << " Underrun=" << underruns40
                  << "; tolerance60 Tracking=" << tracking60 << " Underrun=" << underruns60 << '\n';
    }
}

int main()
{
    try
    {
        test_warmup_and_interpolation();
        test_constant_offsets();
        test_selected_anchor_and_height_offset();
        test_invalid_payload_does_not_mutate_timeline();
        test_duplicates_and_sequence_restart();
        test_stream_identity_and_clock_discontinuities();
        test_underrun_stale_and_long_gap_recovery();
        test_intentional_pause_and_fixed_arrival_mode();
        test_receive_side_storage_bound();
        test_declared_sampling_axis_and_legacy_selection();
        test_declared_contract_rejection_is_transactional();
        test_source_contract_changes_on_stream_or_long_gap();
        test_explicit_modes_ignore_source_declaration();
        test_pre_gmr_pause_resume_availability_budget();
        std::cout << "RealtimeMotionBuffer tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "RealtimeMotionBuffer test failure: " << error.what() << '\n';
        return 1;
    }
}
