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
    gaitController_ = std::make_shared<CGaitController>(node, kinematics_);
    if (servoHandler) {
        servoHandler_ = servoHandler;
    } else {
        servoHandler_ = std::make_shared<CServoHandler>(node);
    }

    if (auto servo_controller = servoHandler_->getServoController()) {
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
}

void CRequester::sendServoRequest(const double duration_s) {
    auto head = kinematics_->getHead();
    auto legs = kinematics_->getLegsAngles();
    servoHandler_->run(CRequest(head, legs, duration_s));
}

void CRequester::onMovementTypeRequest(const MovementRequest& msg) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CRequester::onMovementRequest: " << msg.name);
    gaitController_->setGait(msg);
}

void CRequester::onMovementVelocityRequest(const geometry_msgs::msg::Twist& msg) {
    velocity_ = msg;
}

void CRequester::onMovementBodyPoseRequest(const nikita_interfaces::msg::Pose& msg) {
    poseBody_ = msg;
}

void CRequester::update(std::chrono::milliseconds timeslice) {
    if (gaitController_->updateSelectedGait(velocity_, poseBody_)) {
        double duration_s = double(timeslice.count() / 1000.0);
        sendServoRequest(duration_s);
    }
}

}  // namespace nikita_movement
