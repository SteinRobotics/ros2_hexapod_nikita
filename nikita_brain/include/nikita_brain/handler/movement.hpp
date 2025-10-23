/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"
//
#include "geometry_msgs/msg/twist.hpp"
#include "nikita_interfaces/msg/movement_request.hpp"
//
#include "ihandler.hpp"
#include "requester/irequester.hpp"
#include "requester/simpletimer.hpp"

namespace brain {

class CMovement : public IHandler {
   public:
    CMovement(std::shared_ptr<rclcpp::Node> node);
    virtual ~CMovement() = default;

    void update() override;
    void cancel() override;

    void run(std::shared_ptr<CRequestMove> request);

   private:
    void timerCallback();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<nikita_interfaces::msg::MovementRequest>::SharedPtr pub_;
    std::shared_ptr<CSimpleTimer> simpleTimer_;
};

}  // namespace brain
