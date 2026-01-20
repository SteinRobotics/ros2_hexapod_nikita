/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "handler/movement.hpp"

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

namespace brain {

CMovement::CMovement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    callback_timer_ = std::make_unique<CCallbackTimer>();
    pub_movement_type_ = node_->create_publisher<MovementRequest>("cmd_movement_type", 10);
    pub_body_pose_ = node_->create_publisher<Pose>("cmd_body_pose", 10);
    pub_head_orientation_ = node_->create_publisher<Orientation>("cmd_head_orientation", 10);
    pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void CMovement::run(std::shared_ptr<RequestMovementType> request) {
    setDone(false);
    pub_movement_type_->publish(request->movementRequest);
    callback_timer_->waitSecondsNonBlocking(request->movementRequest.duration_s,
                                            std::bind(&CMovement::timerCallback, this));
}

void CMovement::run(std::shared_ptr<RequestSinglePose> request) {
    pub_body_pose_->publish(request->pose);
}

void CMovement::run(std::shared_ptr<RequestHeadOrientation> request) {
    pub_head_orientation_->publish(request->orientation);
}

void CMovement::run(std::shared_ptr<RequestVelocity> request) {
    pub_cmd_vel_->publish(request->velocity);
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
