#include "SourceTimeline.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using RLController::SourceTimeline;

namespace
{
    constexpr int64_t ms = 1'000'000;
    constexpr int64_t source_epoch = 1'790'000'000'000'000'000LL;
    constexpr int64_t receiver_epoch = 120'000'000'000LL;

    void check(bool condition, const std::string &message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    int64_t require(const std::optional<int64_t> &value, const char *message)
    {
        check(value.has_value(), message);
        return *value;
    }

    void test_integer_epochs_and_offsets()
    {
        // Identical trajectories with hours/days of clock-origin difference must
        // have identical source-relative keys and playback phase.
        for (const int64_t offset : std::array<int64_t, 5>{0, -150 * ms, 250 * ms, -86'400'000 * ms, 86'400'000 * ms})
        {
            SourceTimeline clock;
            for (int i = 0; i <= 30; ++i)
            {
                const int64_t key = require(clock.observe(source_epoch + offset + i * 20 * ms,
                                                          receiver_epoch + i * 20 * ms), "source rejected");
                check(key == i * 20 * ms, "clock origin leaked into stored key");
                if (i < 25) check(!clock.locked(), "clock locked before 500ms warmup");
            }
            check(clock.locked(), "clock did not lock after warmup");
            check(require(clock.source_now(receiver_epoch + 640 * ms), "missing mapped query") == 640 * ms,
                  "constant remote offset changed playback phase");
        }

        SourceTimeline::Config cfg;
        cfg.warmup_ns = 0;
        cfg.min_samples = 2;
        SourceTimeline fine(cfg);
        check(require(fine.observe(source_epoch, receiver_epoch), "first ns sample") == 0, "first key");
        check(require(fine.observe(source_epoch + 1, receiver_epoch + 1), "second ns sample") == 1,
              "integer epoch subtraction lost one nanosecond");
        check(require(fine.source_now(receiver_epoch + 2), "ns query") == 2, "ns query precision");
    }

    void test_warmup_rejection_and_reset()
    {
        SourceTimeline::Config cfg;
        cfg.warmup_ns = 100 * ms;
        cfg.min_samples = 8;
        SourceTimeline clock(cfg);
        for (int i = 0; i < 7; ++i)
            require(clock.observe(source_epoch + i * 20 * ms, receiver_epoch + i * 20 * ms), "warmup input");
        check(!clock.locked(), "min sample count ignored");
        check(!clock.source_now(receiver_epoch + 120 * ms), "unlocked clock returned a valid query");
        require(clock.observe(source_epoch + 140 * ms, receiver_epoch + 140 * ms), "locking input");
        check(clock.locked(), "clock failed to lock after both conditions");

        const auto count = clock.sample_count();
        check(!clock.observe(source_epoch + 140 * ms, receiver_epoch + 160 * ms), "duplicate source accepted");
        check(!clock.observe(source_epoch + 120 * ms, receiver_epoch + 160 * ms), "old source accepted");
        check(!clock.observe(source_epoch + 160 * ms, receiver_epoch + 139 * ms), "old receiver time accepted");
        check(!clock.observe(-1, receiver_epoch + 160 * ms), "negative source accepted");
        check(clock.sample_count() == count, "rejected input changed estimator samples");
        require(clock.observe(source_epoch + 160 * ms, receiver_epoch + 160 * ms), "rejection poisoned next input");
        const int64_t current = require(clock.source_now(receiver_epoch + 180 * ms), "query after reject");
        check(!clock.source_now(receiver_epoch + 170 * ms), "backward query accepted");
        check(require(clock.source_now(receiver_epoch + 200 * ms), "query after backward rejection") > current,
              "backward rejection poisoned query progression");

        clock.reset();
        check(!clock.locked() && clock.sample_count() == 0, "reset did not clear clock state");
        check(!clock.source_now(receiver_epoch + 300 * ms), "reset clock still queryable");
        check(require(clock.observe(source_epoch - 10'000 * ms, receiver_epoch + 300 * ms), "reset origin") == 0,
              "reset did not establish a new source origin");

        cfg.min_samples = 2;
        cfg.warmup_ns = 500 * ms;
        SourceTimeline elapsed(cfg);
        require(elapsed.observe(source_epoch, receiver_epoch), "elapsed first");
        require(elapsed.observe(source_epoch + 20 * ms, receiver_epoch + 20 * ms), "elapsed second");
        check(!elapsed.locked(), "warmup duration ignored despite enough samples");
    }

    void test_jitter_and_phase_slew()
    {
        SourceTimeline::Config cfg;
        cfg.warmup_ns = 0;
        cfg.min_samples = 2;
        cfg.window_ns = 100 * ms;
        cfg.quantile = 0.0;
        SourceTimeline clock(cfg);
        require(clock.observe(source_epoch, receiver_epoch), "slew first");
        require(clock.observe(source_epoch + 20 * ms, receiver_epoch + 20 * ms), "slew second");
        require(clock.source_now(receiver_epoch + 20 * ms), "slew lock query");
        // A sustained 20ms receive queue appears. Once the old minimum ages out,
        // correction must be gradual, not a 20ms discontinuous reference jump.
        for (int i = 2; i <= 12; ++i)
            require(clock.observe(source_epoch + i * 20 * ms, receiver_epoch + (i * 20 + 20) * ms),
                    "queued input rejected");
        const int64_t now = receiver_epoch + 260 * ms;
        const int64_t mapped = require(clock.source_now(now), "slewed query");
        check(std::abs(clock.offset_ns() - 240'000.0) <= 1.0, "phase adjustment exceeded elapsed*0.001");
        check(mapped == 260 * ms - 240'000, "queued offset was applied as a jump");
        check(require(clock.source_now(now), "same-time query") == mapped, "same-time query adjusted phase twice");
        const int64_t next = require(clock.source_now(now + 20 * ms), "next slew query");
        check(next - mapped >= 19'980'000 && next - mapped <= 20'020'000,
              "query rate left allowed +/-0.1 percent interval");

        // The reverse correction must also be bounded and monotonic.
        cfg.quantile = 1.0;
        SourceTimeline reverse(cfg);
        require(reverse.observe(source_epoch, receiver_epoch), "reverse first");
        require(reverse.observe(source_epoch + 20 * ms, receiver_epoch + 40 * ms), "reverse second");
        require(reverse.source_now(receiver_epoch + 40 * ms), "reverse lock query");
        check(reverse.offset_ns() == 20 * ms, "reverse initial phase");
        for (int i = 2; i <= 12; ++i)
            require(reverse.observe(source_epoch + i * 20 * ms, receiver_epoch + i * 20 * ms), "reverse input");
        require(reverse.source_now(receiver_epoch + 240 * ms), "reverse query");
        check(std::abs(reverse.offset_ns() - 19'800'000.0) <= 1.0, "negative phase slew is not bounded");
    }

    void test_small_clock_drift_and_window_bound()
    {
        for (const int drift_sign : {-1, 1})
        {
            SourceTimeline clock;
            std::optional<int64_t> previous;
            int64_t max_error = 0;
            for (int i = 0; i <= 3000; ++i)
            {
                const int64_t source_rel = i * (20 * ms + drift_sign * 4'000LL); // +/-200ppm
                const int64_t recv_rel = i * 20 * ms;
                require(clock.observe(source_epoch + source_rel, receiver_epoch + recv_rel), "drift sample");
                auto query = clock.source_now(receiver_epoch + recv_rel);
                if (!query) continue;
                if (previous)
                    check(*query > *previous && *query - *previous >= 19'980'000 &&
                              *query - *previous <= 20'020'000,
                          "drift query is not monotonic or violates configured rate bound");
                previous = query;
                max_error = std::max(max_error, std::abs(*query - source_rel));
            }
            check(max_error < 3 * ms, "small drift accumulated unbounded playback phase error");
            check(clock.sample_count() <= 501, "10s quantile window did not evict old samples");
        }
    }

    void test_config_validation()
    {
        auto rejects = [](SourceTimeline::Config cfg) {
            try { SourceTimeline invalid(cfg); }
            catch (const std::invalid_argument &) { return true; }
            return false;
        };
        SourceTimeline::Config cfg;
        cfg.min_samples = 1;
        check(rejects(cfg), "one-sample configuration accepted");
        cfg.min_samples = 8193;
        check(rejects(cfg), "unreachable minimum sample count accepted");
        cfg = {};
        cfg.window_ns = cfg.warmup_ns - 1;
        check(rejects(cfg), "window shorter than warmup accepted");
        cfg = {};
        cfg.max_slew = 1.0;
        check(rejects(cfg), "nonmonotonic slew configuration accepted");
        cfg = {};
        cfg.quantile = std::numeric_limits<double>::quiet_NaN();
        check(rejects(cfg), "NaN quantile accepted");
    }
}

int main()
{
    try
    {
        test_integer_epochs_and_offsets();
        test_warmup_rejection_and_reset();
        test_jitter_and_phase_slew();
        test_small_clock_drift_and_window_bound();
        test_config_validation();
        std::cout << "SourceTimeline tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "SourceTimeline test failure: " << error.what() << '\n';
        return 1;
    }
}
