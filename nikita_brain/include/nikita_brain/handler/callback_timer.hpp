/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once
#include <chrono>
#include <functional>
#include <thread>

using namespace std::chrono_literals;

class CCallbackTimer {
   public:
    CCallbackTimer() : isRunning_(false) {
    }

    void start(std::chrono::milliseconds intervalMilliseconds, std::function<void()> callback,
               bool singleShot = false) {
        if (isRunning_) return;

        isRunning_ = true;
        timerThread_ = std::thread([this, intervalMilliseconds, callback, singleShot]() {
            while (isRunning_) {
                std::this_thread::sleep_for(intervalMilliseconds);
                if (isRunning_) {
                    callback();
                    if (singleShot) isRunning_ = false;
                }
            }
        });
        timerThread_.detach();
    }

    void start(uint32_t intervalMilliseconds, std::function<void()> callback, bool singleShot = false) {
        start(std::chrono::milliseconds(intervalMilliseconds), callback, singleShot);
    }
    void startWithoutCallback(std::chrono::milliseconds intervalMilliseconds) {
        start(intervalMilliseconds, []() {}, true);
    }
    void startWithoutCallback(uint32_t intervalMilliseconds) {
        startWithoutCallback(std::chrono::milliseconds(intervalMilliseconds));
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

// int main() {
//     CCallbackTimer timer;

//     // Callback-Funktion
//     auto callback = []() { std::cout << "CCallbackTimer triggered!" << std::endl; };

//     // CCallbackTimer starten
//     std::cout << "Starting timer..." << std::endl;
//     timer.start(1000, callback);  // 1 Sekunde (1000 Millisekunden)

//     // CCallbackTimer nach 5 Sekunden stoppen
//     std::this_thread::sleep_for(std::chrono::seconds(5));
//     std::cout << "Stopping timer..." << std::endl;
//     timer.stop();

//     return 0;
// }
