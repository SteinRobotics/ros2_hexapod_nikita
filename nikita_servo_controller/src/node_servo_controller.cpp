/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#include "rclcpp/rclcpp.hpp"
//
#include "servo_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_servo_controller");

    auto servoController = std::make_shared<ServoController>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}