/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "rclcpp/rclcpp.hpp"
//
#include "geometry_msgs/msg/twist.hpp"
#include "nikita_interfaces/msg/movement_request.hpp"
//
#include <nikita_utils/callback_timer.hpp>

#include "ihandler.hpp"
#include "requester/irequester.hpp"

namespace brain {

class CMovement : public IHandler {
   public:
    CMovement(std::shared_ptr<rclcpp::Node> node);
    virtual ~CMovement() = default;

    void update() override;
    void cancel() override;

    void run(std::shared_ptr<RequestMovementType> request);
    void run(std::shared_ptr<RequestBodyPose> request);
    void run(std::shared_ptr<RequestVelocity> request);

   private:
    void timerCallback();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<nikita_interfaces::msg::MovementRequest>::SharedPtr pub_movement_type_;
    rclcpp::Publisher<nikita_interfaces::msg::Pose>::SharedPtr pub_body_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    std::shared_ptr<CCallbackTimer> callback_timer_;
};

}  // namespace brain
