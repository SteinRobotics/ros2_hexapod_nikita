/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nikita_interfaces/msg/orientation.hpp"
#include "nikita_interfaces/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace utils {

// limitation filter
template <typename T>
T filterChangeRate(const T& last_value, const T& target_value, const T& change_rate) {
    return std::clamp(target_value, last_value - change_rate, last_value + change_rate);
}

// low-pass filter
template <typename T>
T lowPassFilter(const T& last_value, const T& target_value, double alpha) {
    return alpha * target_value + (1.0 - alpha) * last_value;
}

inline geometry_msgs::msg::Vector3 lowPassFilterVector3(const geometry_msgs::msg::Vector3& last_value,
                                                        const geometry_msgs::msg::Vector3& target_value,
                                                        double alpha) {
    geometry_msgs::msg::Vector3 filtered;
    filtered.x = lowPassFilter(last_value.x, target_value.x, alpha);
    filtered.y = lowPassFilter(last_value.y, target_value.y, alpha);
    filtered.z = lowPassFilter(last_value.z, target_value.z, alpha);
    return filtered;
}

inline geometry_msgs::msg::Twist lowPassFilterTwist(const geometry_msgs::msg::Twist& last_value,
                                                    const geometry_msgs::msg::Twist& target_value,
                                                    double alpha) {
    geometry_msgs::msg::Twist filtered;
    filtered.linear = lowPassFilterVector3(last_value.linear, target_value.linear, alpha);
    filtered.angular = lowPassFilterVector3(last_value.angular, target_value.angular, alpha);
    return filtered;
}

inline nikita_interfaces::msg::Orientation lowPassFilterOrientation(
    const nikita_interfaces::msg::Orientation& last_value,
    const nikita_interfaces::msg::Orientation& target_value, double alpha) {
    nikita_interfaces::msg::Orientation filtered;
    filtered.roll = lowPassFilter(last_value.roll, target_value.roll, alpha);
    filtered.pitch = lowPassFilter(last_value.pitch, target_value.pitch, alpha);
    filtered.yaw = lowPassFilter(last_value.yaw, target_value.yaw, alpha);
    return filtered;
}

inline nikita_interfaces::msg::Pose lowPassFilterPose(const nikita_interfaces::msg::Pose& last_value,
                                                      const nikita_interfaces::msg::Pose& target_value,
                                                      double alpha) {
    nikita_interfaces::msg::Pose filtered;
    filtered.position = lowPassFilterVector3(last_value.position, target_value.position, alpha);
    filtered.orientation = lowPassFilterOrientation(last_value.orientation, target_value.orientation, alpha);
    return filtered;
}

}  // namespace utils
