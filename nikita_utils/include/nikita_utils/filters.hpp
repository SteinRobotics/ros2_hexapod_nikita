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
T filterChangeRate(const T& lastValue, const T& targetValue, const T& changeRate) {
    return std::clamp(targetValue, lastValue - changeRate, lastValue + changeRate);
}

// low-pass filter
template <typename T>
T lowPassFilter(const T& lastValue, const T& targetValue, double alpha) {
    return alpha * targetValue + (1.0 - alpha) * lastValue;
}

inline geometry_msgs::msg::Vector3 lowPassFilterVector3(const geometry_msgs::msg::Vector3& lastValue,
                                                        const geometry_msgs::msg::Vector3& targetValue,
                                                        double alpha) {
    geometry_msgs::msg::Vector3 filtered;
    filtered.x = lowPassFilter(lastValue.x, targetValue.x, alpha);
    filtered.y = lowPassFilter(lastValue.y, targetValue.y, alpha);
    filtered.z = lowPassFilter(lastValue.z, targetValue.z, alpha);
    return filtered;
}

inline geometry_msgs::msg::Twist lowPassFilterTwist(const geometry_msgs::msg::Twist& lastValue,
                                                    const geometry_msgs::msg::Twist& targetValue,
                                                    double alpha) {
    geometry_msgs::msg::Twist filtered;
    filtered.linear = lowPassFilterVector3(lastValue.linear, targetValue.linear, alpha);
    filtered.angular = lowPassFilterVector3(lastValue.angular, targetValue.angular, alpha);
    return filtered;
}

inline nikita_interfaces::msg::Orientation lowPassFilterOrientation(
    const nikita_interfaces::msg::Orientation& lastValue,
    const nikita_interfaces::msg::Orientation& targetValue, double alpha) {
    nikita_interfaces::msg::Orientation filtered;
    filtered.roll = lowPassFilter(lastValue.roll, targetValue.roll, alpha);
    filtered.pitch = lowPassFilter(lastValue.pitch, targetValue.pitch, alpha);
    filtered.yaw = lowPassFilter(lastValue.yaw, targetValue.yaw, alpha);
    return filtered;
}

inline nikita_interfaces::msg::Pose lowPassFilterPose(const nikita_interfaces::msg::Pose& lastValue,
                                                      const nikita_interfaces::msg::Pose& targetValue,
                                                      double alpha) {
    nikita_interfaces::msg::Pose filtered;
    filtered.position = lowPassFilterVector3(lastValue.position, targetValue.position, alpha);
    filtered.orientation = lowPassFilterOrientation(lastValue.orientation, targetValue.orientation, alpha);
    return filtered;
}

}  // namespace utils
