#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <nikita_utils/simpletimer.hpp>
#include <thread>

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

TEST(SimpleTimerTest, IsRunningReportsCorrectState) {
    CSimpleTimer timer;

    EXPECT_FALSE(timer.isRunning());

    timer.waitSecondsNonBlocking(0.5, []() {});

    EXPECT_TRUE(timer.isRunning());

    // wait for 0.6 seconds to ensure the timer has completed
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    EXPECT_FALSE(timer.isRunning());
}

TEST(SimpleTimerTest, GetSecondsElapsedReportsCorrectTime) {
    CSimpleTimer timer;
    timer.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    double elapsed = timer.getSecondsElapsed();
    EXPECT_GE(elapsed, 0.25);
    EXPECT_LT(elapsed, 0.3);

    timer.stop();
    EXPECT_EQ(timer.getSecondsElapsed(), 0.0);
}

TEST(SimpleTimerTest, HaveSecondsElapsedWorksCorrectly) {
    CSimpleTimer timer;
    timer.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    EXPECT_TRUE(timer.haveSecondsElapsed(0.2));
    EXPECT_FALSE(timer.haveSecondsElapsed(0.4));

    timer.stop();
    EXPECT_FALSE(timer.haveSecondsElapsed(0.1));
}