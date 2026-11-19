#include "RealtimeMotionBufferVla.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace RLController
{
    namespace
    {
        void check(bool ok, const char *message)
        {
            if (!ok) throw std::runtime_error(message);
        }
        void near(float a, float b, const char *message)
        {
            check(std::abs(a - b) < 1.e-4f, message);
        }
    }

    struct VlaMotionBufferTest
    {
        static constexpr int64_t ms = 1'000'000;
        static RealtimeMotionBufferVla buffer()
        {
            return {20 * ms, 40 * ms, Eigen::VectorXf::Constant(27, -1.f),
                    {0, 0, .61f}, Eigen::Quaternionf::Identity(), 1000 * ms};
        }
        static VlaPosePacket packet(int from, int to)
        {
            VlaPosePacket packet{{}, "publisher-a"};
            for (int i = from; i <= to; ++i)
            {
                VlaPoseFrame f{};
                f.index = i;
                f.position = {float(i), 0, .61f};
                for (size_t j = 0; j < f.joints.size(); ++j)
                    f.joints[j] = float(j) + .01f * i;
                const float half_yaw = .05f * i;
                const float sign = i % 2 == 0 ? 1.f : -1.f;
                f.quaternion = {sign * std::cos(half_yaw), 0, 0, sign * std::sin(half_yaw)};
                packet.frames.push_back(f);
            }
            return packet;
        }

        // A source ramp makes each reference position an independent reading
        // of playback phase, including in the overlap between two action chunks.
        static void test_chunk_recovery()
        {
            constexpr int steps = 273, horizon = 25;
            constexpr int64_t period = 1'000'000'000 / 30;
            std::array<int64_t, steps> sent_at{};
            int64_t start = 0;
            for (int i = 0; i < steps; ++i)
            {
                // main.py charges 120 ms inference to its first action step.
                // The following step can publish after just its 1 ms overhead.
                sent_at[i] = start + ms + (i % horizon == 0 ? 120 * ms : 0);
                start = std::max(start + period, sent_at[i]);
            }
            for (int offset_ms = 0; offset_ms < 20; ++offset_ms)
            {
                for (const bool lose_first_packet : {false, true})
                {
                    auto b = buffer();
                    // Step 0 seeds the publisher window, [0,1] is its first message.
                    int next = 1, latest = -1;
                    double phase = -1;
                    std::array<int, (steps - 1) / horizon> frozen_pairs{};
                    for (int64_t t = offset_ms * ms; t <= sent_at.back() + 180 * ms; t += 20 * ms)
                    {
                        while (next < steps && sent_at[next] <= t)
                        {
                            if (!lose_first_packet || next % horizon != 0)
                            {
                                b.accept_locked(packet(next - 1, next), sent_at[next]);
                                latest = next;
                            }
                            ++next;
                        }
                        auto w = b.sample_locked(t);
                        check(!w.fault && w.ready == (latest >= 4), "unexpected tracking transition");
                        if (!w.ready) continue;
                        const double cap = latest - 3.6; // 120 ms in source-frame units
                        const double previous = phase;
                        phase = previous < 0 ? cap : std::min(cap, previous + .6);
                        for (int s = 0; s < 5; ++s)
                            near(w.frames[s].root_pos_w.x(), static_cast<float>(phase + .6 * s),
                                 "chunk recovery lost or retimed a reference");
                        if (latest % horizon == horizon - 1 && phase == previous)
                            ++frozen_pairs[latest / horizon];
                    }
                    check(latest == steps - 1, "incomplete replay");
                    for (int pairs : frozen_pairs)
                        check(pairs >= 2, "missing steady five-frame window during inference");
                }
            }
        }
        static void run()
        {
            auto b = buffer();
            check(!b.sample_locked(0).ready, "empty startup");
            b.accept_locked(packet(0, 1), 33 * ms);
            auto warm = b.sample_locked(40 * ms);
            check(!warm.fault && !warm.ready && warm.frames[0].dof_pos[0] == -1.f,
                  "only a partial first window");
            // Packet [1,2] was missed. [2,3] recovers both samples.
            b.accept_locked(packet(2, 3), 102 * ms);
            b.accept_locked(packet(3, 4), 139 * ms);
            auto w = b.sample_locked(140 * ms);
            check(w.ready && !w.fault && w.last_frame_index == 4, "complete coverage");
            for (int i = 0; i < 5; ++i)
            {
                const float coordinate = .4f + .6f * i;
                near(w.frames[i].root_pos_w.x(), coordinate, "30 to 50 Hz linear resampling");
                near(w.frames[i].root_pos_w.z(), .61f, "direct base_link height");
                // These are the slots read by the JAKA-order actor, from the
                // deployment fixture's independently verified policy_from_buffer.
                constexpr std::array<int, 27> actor_to_buffer = {
                    0, 3, 8, 13, 17, 21,  // left leg
                    1, 4, 9, 14, 18, 22,  // right leg
                    2,                     // waist
                    5, 10, 15, 19, 23, 25, // left arm
                    7, 12, 16, 20, 24, 26, // right arm
                    6, 11};                // neck
                for (size_t joint = 0; joint < actor_to_buffer.size(); ++joint)
                    near(w.frames[i].dof_pos[actor_to_buffer[joint]],
                         static_cast<float>(joint) + .01f * coordinate, "wire to actor joint order");
                Eigen::Quaternionf expected(Eigen::AngleAxisf(.1f * coordinate, Eigen::Vector3f::UnitZ()));
                near(std::abs(expected.dot(w.frames[i].root_quat_w)), 1.f, "short arc slerp");
            }
            b.accept_locked(packet(3, 4), 250 * ms);
            auto paused = b.sample_locked(320 * ms);
            check(paused.ready && paused.receive_age_ns == 181 * ms, "duplicates must not refresh age");
            for (int i = 0; i < 5; ++i)
                check(w.frames[i].root_pos_w == paused.frames[i].root_pos_w &&
                      w.frames[i].dof_pos == paused.frames[i].dof_pos &&
                      w.frames[i].root_quat_w.coeffs() == paused.frames[i].root_quat_w.coeffs(),
                      "complete window held at cap during pause");
            b.accept_locked(packet(4, 5), 333 * ms);
            auto resumed = b.sample_locked(340 * ms);
            near(resumed.frames[0].root_pos_w.x(), 1.f, "resume by elapsed control interval");
            auto same = b.sample_locked(340 * ms);
            near(same.frames[0].root_pos_w.x(), 1.f, "same-time repeat sample");
            b.accept_locked(packet(7, 8), 370 * ms);  // missing 6, cannot recover from this window
            auto hold = b.sample_locked(375 * ms);
            check(hold.fault && hold.ready, "unrecoverable gap must not be interpolated");
            for (const auto &f : hold.frames)
                check(f.root_pos_w == resumed.frames[0].root_pos_w &&
                      f.dof_pos == resumed.frames[0].dof_pos &&
                      f.root_quat_w.coeffs() == resumed.frames[0].root_quat_w.coeffs(),
                      "hold last issued reference without future motion or default-pose jump");
            b.accept_locked(packet(5, 6), 380 * ms);
            check(b.sample_locked(390 * ms).fault, "fault stays latched until FSM re-entry");

            auto restart = buffer();
            restart.accept_locked(packet(0, 1), 1);
            auto changed = packet(0, 1);
            changed.stream_id = "publisher-b";
            restart.accept_locked(changed, 20 * ms);
            check(restart.sample_locked(20 * ms).fault, "reject cross-session origin");
            auto reentered = buffer();
            changed = packet(0, 4);
            changed.stream_id = "publisher-b";
            reentered.accept_locked(changed, 3000 * ms);
            auto new_session = reentered.sample_locked(3000 * ms);
            check(new_session.ready && !new_session.fault && new_session.last_frame_index == 4,
                  "fresh FSM entry accepts a restarted publisher starting from zero");
            auto overlap = buffer();
            overlap.accept_locked(packet(0, 3), 0);
            auto revision = packet(3, 4);
            revision.frames[0].position[2] += .1f;
            overlap.accept_locked(revision, 10 * ms);
            check(overlap.sample_locked(20 * ms).fault, "inconsistent reference at chunk overlap");
            auto stale = buffer();
            stale.accept_locked(packet(0, 3), 0);
            stale.accept_locked(packet(3, 4), 1001 * ms);
            check(stale.sample_locked(1001 * ms).fault, "stale cannot be hidden by new data");

            auto late = buffer();
            auto high = packet(0, 4);
            for (auto &f : high.frames) f.index += std::numeric_limits<int64_t>::max() - 4;
            late.accept_locked(high, 0);
            auto late_window = late.sample_locked(0);
            check(late_window.ready, "late subscribe at large absolute indices");
            near(late_window.frames[0].root_pos_w.x(), .4f, "relative index arithmetic");
            auto backlog = buffer();
            for (int i = 0; i <= 128; i += 2)
                backlog.accept_locked(packet(i, i + 1), i * ms);
            check(backlog.sample_locked(200 * ms).fault &&
                  backlog.frames_.size() <= RealtimeMotionBufferVla::kCapacity, "bounded storage");
            test_chunk_recovery();
        }
    };
}

int main()
{
    try { RLController::VlaMotionBufferTest::run(); }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
    return 0;
}
