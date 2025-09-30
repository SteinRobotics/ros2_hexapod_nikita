/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <functional>

#include "rclcpp/rclcpp.hpp"
//
#include "nikita_interfaces/msg/servo_index.hpp"
#include "nikita_interfaces/msg/servo_request.hpp"
//
#include "callback_timer.hpp"
#include "requester/requests.hpp"

class CServoHandler {
   public:
    CServoHandler(std::shared_ptr<rclcpp::Node> node);
    virtual ~CServoHandler() = default;

    void cancel();
    bool isDone();
    void setDone(bool state);

    void run(std::shared_ptr<CRequestSendDuration> request);
    void run(std::shared_ptr<CRequestLeg> request);
    void run(std::shared_ptr<CRequestLegs> request);
    void run(std::shared_ptr<CRequestHead> request);

    void setOnDoneCallback(std::function<void()> cb);

   private:
    void timerCallback();

    std::shared_ptr<rclcpp::Node> node_;
    std::map<uint32_t, float> jointAngles_;
    rclcpp::Publisher<nikita_interfaces::msg::ServoRequest>::SharedPtr pubServoRequest_;
    nikita_interfaces::msg::ServoRequest msgServoRequest_;
    std::shared_ptr<CCallbackTimer> callbackTimer_;

    std::function<void()> doneCallback_;

    bool isDone_ = true;
};
