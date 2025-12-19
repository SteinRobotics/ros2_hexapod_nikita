/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
//
#include "geometry_msgs/msg/twist.hpp"
#include "nikita_interfaces/msg/movement_request.hpp"
#include "nikita_interfaces/msg/pose.hpp"
#include "nikita_interfaces/msg/servo_index.hpp"
//
#include "gaitcontroller.hpp"
#include "handler/servohandler.hpp"
#include "igaits.hpp"
#include "kinematics.hpp"

namespace nikita_movement {

class CRequester {
   public:
    CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servoHandler = nullptr);
    virtual ~CRequester() = default;

    void update(std::chrono::milliseconds timeslice);

    void onMovementTypeRequest(const nikita_interfaces::msg::MovementRequest& msg);
    void onMovementVelocityRequest(const geometry_msgs::msg::Twist& msg);
    void onMovementBodyPoseRequest(const nikita_interfaces::msg::Pose& msg);

   private:
    rclcpp::Subscription<nikita_interfaces::msg::MovementRequest>::SharedPtr m_subMovementRequest;

    void sendServoRequest(const double duration_s);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    std::shared_ptr<CGaitController> gaitController_;
    std::shared_ptr<CServoHandler> servoHandler_;

    rclcpp::Subscription<nikita_interfaces::msg::MovementRequest>::SharedPtr subMovementTypeRequest_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subMovementVelocityRequest_;
    rclcpp::Subscription<nikita_interfaces::msg::Pose>::SharedPtr subMovementBodyPoseRequest_;

    geometry_msgs::msg::Twist velocity_;
    nikita_interfaces::msg::Pose poseBody_;
};
}  // namespace nikita_movement
