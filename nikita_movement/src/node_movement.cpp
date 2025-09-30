/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
//
#include "action/action_executor.hpp"
#include "requester/requester.hpp"

constexpr double REFRESH_RATE_HZ = 10;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_movement");
    auto actionExecutor = std::make_shared<CActionExecutor>(node);

    auto requester = std::make_shared<CRequester>(node, actionExecutor);

    rclcpp::Rate loop_rate(REFRESH_RATE_HZ);
    auto timeslice_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / REFRESH_RATE_HZ));

    while (rclcpp::ok()) {
        requester->update(timeslice_ms);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    return 0;
}
