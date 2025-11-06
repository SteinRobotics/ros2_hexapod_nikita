/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <functional>

#include "rclcpp/rclcpp.hpp"
//
#include "nikita_interfaces/msg/servo_index.hpp"
//
#include <nikita_utils/simpletimer.hpp>

#include "requester/kinematics.hpp"
#include "servo_controller.hpp"

class CRequest {
   public:
    CRequest() = default;
    CRequest(CHead head, std::map<ELegIndex, CLegAngles> legAngles, double duration)
        : head_(head), legAngles_(legAngles), duration_(duration) {};

    virtual ~CRequest() = default;

    CHead head() const {
        return head_;
    }
    std::map<ELegIndex, CLegAngles> legAngles() const {
        return legAngles_;
    }
    double duration() const {
        return duration_;
    }

   private:
    CHead head_;
    std::map<ELegIndex, CLegAngles> legAngles_;
    double duration_ = double(0);
};

class CServoHandler {
   public:
    CServoHandler(std::shared_ptr<rclcpp::Node> node);
    virtual ~CServoHandler() = default;

    virtual void requestWithoutQueue(CRequest request);
    virtual void appendRequest(CRequest request);
    bool isDone();

   private:
    void run(CRequest request, bool blocking = true);
    void executeNextPendingRequest();
    void cancelRunningRequest();
    void timerCallback();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CServoController> servoController_;
    std::shared_ptr<CSimpleTimer> simpleTimer_;
    std::list<CRequest> pendingRequests_;
    bool isDone_ = true;
};
