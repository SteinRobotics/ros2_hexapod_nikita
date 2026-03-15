/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "handler/movement.hpp"

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

namespace brain {

CMovement::CMovement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    callback_timer_ = std::make_unique<CCallbackTimer>();
    pub_cmd_movement_ = node_->create_publisher<MovementRequest>("cmd_movement", 10);
}

void CMovement::publish() {
    current_request_.header.stamp = node_->get_clock()->now();
    pub_cmd_movement_->publish(current_request_);
}

void CMovement::run(std::shared_ptr<RequestMovementType> request) {
    setDone(false);
    current_request_.type = request->movementRequest.type;
    current_request_.name = request->movementRequest.name;
    current_request_.direction = request->movementRequest.direction;
    current_request_.duration_s = request->movementRequest.duration_s;
    publish();
    callback_timer_->waitSecondsNonBlocking(request->movementRequest.duration_s,
                                            std::bind(&CMovement::timerCallback, this));
}

void CMovement::run(std::shared_ptr<RequestSinglePose> request) {
    current_request_.body_pose = request->pose;
    publish();
}

void CMovement::run(std::shared_ptr<RequestHeadOrientation> request) {
    current_request_.head_orientation = request->orientation;
    publish();
}

void CMovement::run(std::shared_ptr<RequestVelocity> request) {
    current_request_.velocity = request->velocity;
    publish();
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
