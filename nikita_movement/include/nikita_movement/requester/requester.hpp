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
#include "actionpackagesparser.hpp"
#include "gaitcontroller.hpp"
#include "handler/servohandler.hpp"
#include "igaits.hpp"
#include "kinematics.hpp"

class CRequester {
   public:
    CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servoHandler = nullptr);
    virtual ~CRequester() = default;

    void update(std::chrono::milliseconds timeslice);

    // void onServoStatus(const nikita_interfaces::msg::ServoStatus& msg);
    void onMovementTypeRequest(const nikita_interfaces::msg::MovementRequest& msg);
    void onMovementVelocityRequest(const geometry_msgs::msg::Twist& msg);
    void onMovementBodyPoseRequest(const nikita_interfaces::msg::Pose& msg);

    // test helper: allow injection of a custom servo handler (mock)
    // NOTE: dependencies (like CServoHandler) should be injected via constructor.

   private:
    rclcpp::Subscription<nikita_interfaces::msg::MovementRequest>::SharedPtr m_subMovementRequest;

    void initRequestHandlers();
    void requestGait(const nikita_interfaces::msg::MovementRequest& msg);
    void requestDance(const nikita_interfaces::msg::MovementRequest& msg);
    void requestBite(const nikita_interfaces::msg::MovementRequest& msg);
    void requestStomp(const nikita_interfaces::msg::MovementRequest& msg);
    void requestClap(const nikita_interfaces::msg::MovementRequest& msg);
    // the following requests are used for calibration and testing and are not used for normal operation
    void requestTestBody(const nikita_interfaces::msg::MovementRequest& msg);
    void requestTestLegs(const nikita_interfaces::msg::MovementRequest& msg);

    void requestNeutral(const nikita_interfaces::msg::MovementRequest& msg);
    void requestCalibrate(const nikita_interfaces::msg::MovementRequest& msg);

    void sendServoRequest(const double duration_s);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    std::shared_ptr<CGaitController> gaitController_;
    std::shared_ptr<CActionPackagesParser> actionPackagesParser_;
    std::shared_ptr<CServoHandler> servoHandler_;

    rclcpp::Subscription<nikita_interfaces::msg::MovementRequest>::SharedPtr subMovementTypeRequest_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subMovementVelocityRequest_;
    rclcpp::Subscription<nikita_interfaces::msg::Pose>::SharedPtr subMovementBodyPoseRequest_;

    // rclcpp::Subscription<nikita_interfaces::msg::ServoStatus>::SharedPtr subServoStatus_;
    uint8_t activeRequest_ = nikita_interfaces::msg::MovementRequest::NO_REQUEST;
    std::unordered_map<uint8_t, std::function<void(const nikita_interfaces::msg::MovementRequest&)>>
        request_handlers_;
    geometry_msgs::msg::Twist velocity_;
    nikita_interfaces::msg::Pose poseBody_;
};
