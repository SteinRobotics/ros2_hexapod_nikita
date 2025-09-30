#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

class CSimpleTimer {
   public:
    CSimpleTimer(bool automaticStart = false) {
        if (automaticStart) {
            start();
        }
    }
    ~CSimpleTimer() {
        stop();
    }

    void start() {
        startTime_ = std::chrono::steady_clock::now();
        isRunning_ = true;
    }

    void stop() {
        isRunning_ = false;
    }

    bool isRunning() const {
        return isRunning_;
    }

    uint64_t getMsElapsed() const {
        if (!isRunning_) return 0;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime_);
        return static_cast<uint64_t>(elapsed.count());
    }

    bool haveMsElapsed(uint64_t ms) const {
        return getMsElapsed() >= ms;
    }

    void waitMsNonBlocking(uint64_t ms, std::function<void()> callback) {
        start();
        std::thread([ms, callback]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            callback();
        }).detach();
        stop();
    }

    void waitMsBlocking(uint64_t ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

   private:
    std::chrono::steady_clock::time_point startTime_;
    std::atomic<bool> isRunning_ = false;
};
