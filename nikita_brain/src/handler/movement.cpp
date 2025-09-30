/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "handler/movement.hpp"

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

namespace brain {

CMovement::CMovement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    callbackTimer_ = std::make_unique<CCallbackTimer>();
    pub_ = node_->create_publisher<MovementRequest>("movement_request", 10);
}

void CMovement::run(std::shared_ptr<CRequestMove> request) {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "CMovement::run RequestMove |" << request->movementRequest().name);
    setDone(false);
    // auto msg = MovementRequest();
    // msg.request_type = request->movementType();
    // msg.name = getMovementTypeName(msg.request_type);
    // msg.velocity = request->velocity();
    // msg.duration_ms = uint32_t(request->minDuration() * 1000);
    pub_->publish(request->movementRequest());
    callbackTimer_->start(request->movementRequest().duration_ms, std::bind(&CMovement::timerCallback, this),
                          true);
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
