// Copyright (c) 2026 Spaaaace-yyj
// SPDX-License-Identifier: MIT
#include "../src/frame_rate_limiter.hpp"

#include <cstdlib>
#include <iostream>

using mindvision_camera::FrameRateLimiter;

void require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

int main()
{
    FrameRateLimiter limiter;
    const int64_t epoch = 1790774165000000000LL;

    limiter.configure(true, 30);
    for (int i = 0; i < 1000; ++i)
        require(limiter.accept(epoch + i * 10000000LL), "full-speed frame dropped");

    for (int input_hz : {100, 200})
    {
        limiter.configure(false, 30);
        int count = 0;
        for (int i = 0; i < input_hz * 10; ++i)
            count += limiter.accept(epoch + i * (1000000000LL / input_hz));
        require(count == 300, "30 Hz average was not maintained");
    }

    limiter.configure(false, 30);
    for (int i = 0; i < 200; ++i)
        require(limiter.accept(epoch + i * 50000000LL), "low-rate input dropped");

    limiter.configure(false, 30);
    require(limiter.accept(epoch), "first frame dropped");
    require(!limiter.accept(epoch), "duplicate timestamp accepted");
    require(!limiter.accept(epoch + 10000000LL), "early frame accepted");
    require(limiter.accept(epoch - 1000000000LL), "clock reset not recovered");
    require(limiter.accept(epoch + 3600000000000LL), "large gap not recovered");
    require(!limiter.accept(epoch + 3600000000001LL), "burst after large gap");

    std::cout << "frame_rate_limiter: all checks passed\n";
}
