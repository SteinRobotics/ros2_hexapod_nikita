/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once
#include <chrono>
#include <functional>
#include <thread>

class CCallbackTimer {
   public:
    CCallbackTimer() : isRunning_(false) {
    }

    void start(uint32_t intervalMilliseconds, std::function<void()> callback, bool singleShot = false) {
        if (isRunning_) {
            std::cout << "CCallbackTimer:: start | Timer is already running. Stopping it first." << std::endl;
            stop();
        }
        isRunning_ = true;
        timerThread_ = std::thread([this, intervalMilliseconds, callback, singleShot]() {
            while (isRunning_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(intervalMilliseconds));
                if (isRunning_) {
                    if (singleShot) isRunning_ = false;
                    callback();
                }
            }
        });
        timerThread_.detach();
    }

    void startWithoutCallback(uint32_t intervalMilliseconds) {
        start(intervalMilliseconds, []() {}, true);
    }

    void stop() {
        isRunning_ = false;
        if (timerThread_.joinable()) {
            timerThread_.join();
        }
    }

    bool isRunning() {
        return isRunning_;
    }

    ~CCallbackTimer() {
        stop();
    }

   private:
    std::thread timerThread_;
    std::atomic<bool> isRunning_;
};
