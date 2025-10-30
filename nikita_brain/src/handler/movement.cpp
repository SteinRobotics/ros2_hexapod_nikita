/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "handler/movement.hpp"

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

namespace brain {

CMovement::CMovement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    simpleTimer_ = std::make_unique<CSimpleTimer>();
    pub_ = node_->create_publisher<MovementRequest>("movement_request", 10);
}

void CMovement::run(std::shared_ptr<CRequestMove> request) {
    // RCLCPP_INFO_STREAM(node_->get_logger(),
    //                    "CMovement::run RequestMove |" << request->movementRequest().name);
    setDone(false);
    pub_->publish(request->movementRequest());
    simpleTimer_->waitSecondsNonBlocking(request->movementRequest().duration_s,
                                         std::bind(&CMovement::timerCallback, this));
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
