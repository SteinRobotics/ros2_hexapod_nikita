#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "requester/simpletimer.hpp"

TEST(SimpleTimerTest, WaitSecondsBlockingCallsCallback) {
    CSimpleTimer timer;
    auto t0 = std::chrono::steady_clock::now();
    timer.waitSecondsBlocking(0.2);
    auto t1 = std::chrono::steady_clock::now();

    std::chrono::duration<double> dur = t1 - t0;
    // greater than or equal to 0.2 seconds
    EXPECT_GE(dur.count(), 0.2);
    // less than 0.3 seconds to allow for some scheduling jitter
    EXPECT_LT(dur.count(), 0.3);
}

TEST(SimpleTimerTest, WaitSecondsNonBlockingCallsCallback) {
    CSimpleTimer timer;
    std::atomic<bool> called{false};

    auto t0 = std::chrono::steady_clock::now();

    timer.waitSecondsNonBlocking(0.2, [&called]() { called = true; });

    // Immediately after scheduling, the timer should report running
    EXPECT_TRUE(timer.isRunning());

    // wait up to 1 seconds for the callback to be invoked
    bool success = false;
    for (int i = 0; i < 100; ++i) {
        if (called.load()) {
            success = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> dur = t1 - t0;

    EXPECT_TRUE(success);
    // After the callback ran the timer should have been stopped by the implementation
    EXPECT_FALSE(timer.isRunning());
    // greater than or equal to 0.2 seconds
    EXPECT_GE(dur.count(), 0.2);
    // less than 0.3 seconds to allow for some scheduling jitter
    EXPECT_LT(dur.count(), 0.3);
}
