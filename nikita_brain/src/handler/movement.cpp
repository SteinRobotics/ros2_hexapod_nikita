/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "handler/movement.hpp"

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

namespace brain {

CMovement::CMovement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    simpleTimer_ = std::make_unique<CSimpleTimer>();
    pubMovementType_ = node_->create_publisher<MovementRequest>("movement_request", 10);
    pubBodyPose_ = node_->create_publisher<Pose>("body_pose_request", 10);
    pubCmdVel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void CMovement::run(std::shared_ptr<CRequestMovementType> request) {
    // RCLCPP_INFO_STREAM(node_->get_logger(),
    //                    "CMovement::run RequestMove |" << request->movementRequest().name);
    setDone(false);
    pubMovementType_->publish(request->movementRequest());
    simpleTimer_->waitSecondsNonBlocking(request->movementRequest().duration_s,
                                         std::bind(&CMovement::timerCallback, this));
}

void CMovement::run(std::shared_ptr<CRequestMoveBody> request) {
    // RCLCPP_INFO_STREAM(node_->get_logger(),
    //                    "CMovement::run RequestMoveBody |" << request->movementRequest().name);
    // setDone(false);
    pubBodyPose_->publish(request->pose());
    // simpleTimer_->waitSecondsNonBlocking(request->pose().duration_s,
    //  std::bind(&CMovement::timerCallback, this));
}

void CMovement::run(std::shared_ptr<CRequestMoveVelocity> request) {
    // RCLCPP_INFO_STREAM(node_->get_logger(),
    //                    "CMovement::run RequestMoveVelocity |" << request->movementRequest().name);
    // setDone(false);
    pubCmdVel_->publish(request->velocity());
    // simpleTimer_->waitSecondsNonBlocking(request->velocity().duration_s,
    //  std::bind(&CMovement::timerCallback, this));
}

void CMovement::timerCallback() {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CMovement::timerCallback");
    // TODO better trigger callback to request_executor::execute
    setDone(true);
}

void CMovement::cancel() {
}

void CMovement::update() {
}

}  // namespace brain
