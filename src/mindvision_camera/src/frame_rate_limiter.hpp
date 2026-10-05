// Copyright (c) 2026 Spaaaace-yyj
// SPDX-License-Identifier: MIT
#ifndef MINDVISION_CAMERA_FRAME_RATE_LIMITER_HPP_
#define MINDVISION_CAMERA_FRAME_RATE_LIMITER_HPP_

#include <cstdint>

namespace mindvision_camera
{
// Advance a fixed acquisition-time grid instead of resetting it after each
// accepted frame (which would turn a 100 Hz -> 30 Hz request into 25 Hz).
class FrameRateLimiter
{
public:
    void configure(bool full_speed, int target_fps)
    {
        period_ns_ = full_speed ? 0 : 1000000000LL / target_fps;
        initialized_ = false;
    }

    bool accept(int64_t stamp_ns)
    {
        if (period_ns_ == 0)
            return true;

        if (!initialized_ || stamp_ns < last_stamp_ns_)
        {
            initialized_ = true;
            last_stamp_ns_ = stamp_ns;
            next_stamp_ns_ = stamp_ns + period_ns_;
            return true;
        }

        last_stamp_ns_ = stamp_ns;
        if (stamp_ns < next_stamp_ns_)
            return false;

        // O(1), even after a long gap. Do not burst-publish missed frames.
        const int64_t steps = (stamp_ns - next_stamp_ns_) / period_ns_ + 1;
        next_stamp_ns_ += steps * period_ns_;
        return true;
    }

private:
    int64_t period_ns_{0};
    int64_t last_stamp_ns_{0};
    int64_t next_stamp_ns_{0};
    bool initialized_{false};
};
}  // namespace mindvision_camera

#endif  // MINDVISION_CAMERA_FRAME_RATE_LIMITER_HPP_
