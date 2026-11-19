#pragma once
#include "RealtimeMotionBuffer.hpp"

namespace RLController
{
    struct VlaMotionWindow
    {
        bool ready = false;
        bool fault = false;
        std::array<InterpolatedFrame, 5> frames{};
        const char *fault_reason = "injected VLA fault";
        int64_t last_frame_index = -1;
        int64_t receive_age_ns = -1;
    };

    class RealtimeMotionBufferVla
    {
    public:
        static inline VlaMotionWindow next;
        static inline int sample_calls = 0;
        template<class... Args> explicit RealtimeMotionBufferVla(Args&&...) {}
        VlaMotionWindow sample() { ++sample_calls; return next; }
    };
}
