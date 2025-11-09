/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "handler/movement.hpp"

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

namespace brain {

CMovement::CMovement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    callbackTimer_ = std::make_unique<CCallbackTimer>();
    pubMovementType_ = node_->create_publisher<MovementRequest>("cmd_movement_type", 10);
    pubBodyPose_ = node_->create_publisher<Pose>("cmd_body_pose", 10);
    pubCmdVel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void CMovement::run(std::shared_ptr<CRequestMovementType> request) {
    setDone(false);
    pubMovementType_->publish(request->movementRequest());
    callbackTimer_->waitSecondsNonBlocking(request->movementRequest().duration_s,
                                           std::bind(&CMovement::timerCallback, this));
}

void CMovement::run(std::shared_ptr<CRequestMoveBody> request) {
    pubBodyPose_->publish(request->pose());
}

void CMovement::run(std::shared_ptr<CRequestMoveVelocity> request) {
    pubCmdVel_->publish(request->velocity());
}

void CMovement::timerCallback() {
    // TODO better trigger callback to request_executor::execute
    setDone(true);
}

void CMovement::cancel() {
}

void CMovement::update() {
}

}  // namespace brain
