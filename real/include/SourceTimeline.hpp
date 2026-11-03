#ifndef JAKA_SOURCE_TIMELINE_HPP
#define JAKA_SOURCE_TIMELINE_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

namespace RLController
{
    // Source keys never move. All callers hold the enclosing buffer mutex.
    // This estimates a playback phase, not one-way latency or precision clock sync.
    class SourceTimeline
    {
    public:
        struct Config
        {
            int64_t warmup_ns = 500'000'000;
            size_t min_samples = 8;
            int64_t window_ns = 10'000'000'000LL;
            double quantile = 0.05;
            double max_slew = 0.001; // at most 1 ms phase correction per real second
        };

        SourceTimeline() : SourceTimeline(Config{}) {}
        explicit SourceTimeline(Config config) : config_(config)
        {
            if (config_.warmup_ns < 0 || config_.min_samples < 2 || config_.min_samples > 8192 ||
                config_.warmup_ns > 3'600'000'000'000LL ||
                config_.window_ns < config_.warmup_ns || config_.window_ns <= 0 ||
                config_.window_ns > 86'400'000'000'000LL ||
                !std::isfinite(config_.quantile) || config_.quantile < 0 || config_.quantile > 1 ||
                !std::isfinite(config_.max_slew) || config_.max_slew < 0 || config_.max_slew >= 1)
                throw std::invalid_argument("Invalid SourceTimeline configuration");
        }

        void reset()
        {
            started_ = locked_ = false;
            source_origin_ns_ = recv_origin_ns_ = last_source_ns_ = last_recv_ns_ = 0;
            last_query_ns_ = 0;
            offset_ns_ = target_offset_ns_ = 0;
            offsets_.clear();
        }

        std::optional<int64_t> observe(int64_t source_ns, int64_t recv_ns)
        {
            if (source_ns < 0 || recv_ns < 0 ||
                (started_ && (source_ns <= last_source_ns_ || recv_ns < last_recv_ns_)))
                return std::nullopt;
            if (!started_)
            {
                started_ = true;
                source_origin_ns_ = source_ns;
                recv_origin_ns_ = recv_ns;
            }
            // Subtract integer epochs BEFORE any conversion to floating point.
            const int64_t source_rel = source_ns - source_origin_ns_;
            const int64_t recv_rel = recv_ns - recv_origin_ns_;
            offsets_.push_back({recv_rel, recv_rel - source_rel});
            while (!offsets_.empty() && recv_rel - offsets_.front().first > config_.window_ns)
                offsets_.pop_front();
            // Bound memory even for a malformed very high-frequency publisher.
            if (offsets_.size() > 8192) offsets_.pop_front();
            if (offsets_.size() >= config_.min_samples)
            {
                std::vector<int64_t> values;
                values.reserve(offsets_.size());
                for (const auto &sample : offsets_) values.push_back(sample.second);
                const size_t index = static_cast<size_t>(config_.quantile * (values.size() - 1));
                std::nth_element(values.begin(), values.begin() + index, values.end());
                target_offset_ns_ = static_cast<double>(values[index]);
                if (!locked_ && recv_rel >= config_.warmup_ns)
                {
                    offset_ns_ = target_offset_ns_;
                    last_query_ns_ = recv_ns;
                    locked_ = true;
                }
            }
            last_source_ns_ = source_ns;
            last_recv_ns_ = recv_ns;
            return source_rel;
        }

        std::optional<int64_t> source_now(int64_t now_ns)
        {
            if (!locked_ || now_ns < last_query_ns_) return std::nullopt;
            const int64_t elapsed = now_ns - last_query_ns_;
            const double bound = config_.max_slew * static_cast<double>(elapsed);
            offset_ns_ += std::clamp(target_offset_ns_ - offset_ns_, -bound, bound);
            last_query_ns_ = now_ns;
            return (now_ns - recv_origin_ns_) - static_cast<int64_t>(std::llround(offset_ns_));
        }

        bool locked() const { return locked_; }
        size_t sample_count() const { return offsets_.size(); }
        double offset_ns() const { return offset_ns_; }

    private:
        Config config_;
        bool started_ = false;
        bool locked_ = false;
        int64_t source_origin_ns_ = 0, recv_origin_ns_ = 0;
        int64_t last_source_ns_ = 0, last_recv_ns_ = 0, last_query_ns_ = 0;
        double offset_ns_ = 0, target_offset_ns_ = 0;
        std::deque<std::pair<int64_t, int64_t>> offsets_;
    };
}
#endif
