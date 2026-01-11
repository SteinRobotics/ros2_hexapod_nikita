/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/requester.hpp"

using namespace nikita_interfaces::msg;
using std::placeholders::_1;

namespace nikita_movement {

CRequester::CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servoHandler)
    : node_(node) {
    kinematics_ = std::make_shared<CKinematics>(node);
    gait_controller_ = std::make_shared<CGaitController>(node, kinematics_);
    if (servoHandler) {
        servo_handler_ = servoHandler;
    } else {
        servo_handler_ = std::make_shared<CServoHandler>(node);
    }

    if (auto servo_controller = servo_handler_->getServoController()) {
        servo_controller->setInitialAnglesCallback(
            [this](const std::map<ELegIndex, CLegAngles>& initial_angles) {
                RCLCPP_INFO_STREAM(node_->get_logger(),
                                   "on Callback: initial servo angles received, setting kinematics.");
                for (const auto& [leg_index, leg_angles] : initial_angles) {
                    kinematics_->setLegAngles(leg_index, leg_angles);
                }
            });
    }

    subMovementTypeRequest_ = node_->create_subscription<MovementRequest>(
        "cmd_movement_type", 10, std::bind(&CRequester::onMovementTypeRequest, this, _1));

    subMovementVelocityRequest_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&CRequester::onMovementVelocityRequest, this, _1));

    subMovementBodyPoseRequest_ = node_->create_subscription<nikita_interfaces::msg::Pose>(
        "cmd_body_pose", 10, std::bind(&CRequester::onMovementBodyPoseRequest, this, _1));

    subMovementHeadOrientationRequest_ = node_->create_subscription<nikita_interfaces::msg::Orientation>(
        "cmd_head_orientation", 10, std::bind(&CRequester::onMovementHeadOrientationRequest, this, _1));
}

void CRequester::sendServoRequest(const double duration_s) {
    auto head = kinematics_->getHead();
    auto legs = kinematics_->getLegsAngles();
    servo_handler_->run(CRequest(head, legs, duration_s));
}

void CRequester::onMovementTypeRequest(const MovementRequest& msg) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CRequester::onMovementRequest: " << msg.name);
    gait_controller_->setGait(msg);
}

void CRequester::onMovementVelocityRequest(const geometry_msgs::msg::Twist& msg) {
    velocity_ = msg;
}

void CRequester::onMovementBodyPoseRequest(const nikita_interfaces::msg::Pose& msg) {
    pose_body_ = msg;
}

void CRequester::onMovementHeadOrientationRequest(const nikita_interfaces::msg::Orientation& msg) {
    orientation_head_ = msg;
}

void CRequester::update(std::chrono::milliseconds timeslice) {
    if (gait_controller_->updateSelectedGait(velocity_, pose_body_, orientation_head_)) {
        double duration_s = double(timeslice.count() / 1000.0);
        sendServoRequest(duration_s);
    }
}

}  // namespace nikita_movement
