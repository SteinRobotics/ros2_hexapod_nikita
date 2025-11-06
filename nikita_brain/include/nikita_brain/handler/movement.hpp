/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"
//
#include "geometry_msgs/msg/twist.hpp"
#include "nikita_interfaces/msg/movement_request.hpp"
//
#include <nikita_utils/simpletimer.hpp>

#include "ihandler.hpp"
#include "requester/irequester.hpp"

namespace brain {

class CMovement : public IHandler {
   public:
    CMovement(std::shared_ptr<rclcpp::Node> node);
    virtual ~CMovement() = default;

    void update() override;
    void cancel() override;

    void run(std::shared_ptr<CRequestMovementType> request);
    void run(std::shared_ptr<CRequestMoveBody> request);
    void run(std::shared_ptr<CRequestMoveVelocity> request);

   private:
    void timerCallback();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<nikita_interfaces::msg::MovementRequest>::SharedPtr pubMovementType_;
    rclcpp::Publisher<nikita_interfaces::msg::Pose>::SharedPtr pubBodyPose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;

    std::shared_ptr<CSimpleTimer> simpleTimer_;
};

}  // namespace brain
