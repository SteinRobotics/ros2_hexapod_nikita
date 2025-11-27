/**
 * Minimal mock of CServoHandler for unit tests.
 * It stores CServoRequest objects pushed via appendRequest/requestWithoutQueue
 * and exposes them through getRequests().
 */
#pragma once

#include <list>
#include <memory>

#include "handler/servohandler.hpp"

class CServoHandlerMock : public CServoHandler {
   public:
    CServoHandlerMock(std::shared_ptr<rclcpp::Node> node) : CServoHandler(node) {
    }
    virtual ~CServoHandlerMock() = default;

    void run(CRequest request) override {
        requests_.push_back(std::make_shared<CRequest>(request));
    }

    std::list<std::shared_ptr<CRequest>> getRequests() const {
        return requests_;
    }

    void clear() {
        requests_.clear();
    }

   private:
    std::list<std::shared_ptr<CRequest>> requests_;
};
