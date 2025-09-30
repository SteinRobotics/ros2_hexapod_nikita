/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/lowpassfilter.hpp"

using namespace nikita_interfaces::msg;

// limitation filter
template <typename T>
static T filterChangeRate(const T& lastValue, const T& targetValue, const T& changeRate) {
    return std::clamp(targetValue, lastValue - changeRate, lastValue + changeRate);
}

// low-pass filter
template <typename T>
static T lowPassFilterScalar(const T& lastValue, const T& targetValue, double alpha) {
    return alpha * targetValue + (1.0 - alpha) * lastValue;
}

// Helper for low-pass filtering a Vector3
geometry_msgs::msg::Vector3 lowPassFilterVector3(const geometry_msgs::msg::Vector3& lastValue,
                                                 const geometry_msgs::msg::Vector3& targetValue,
                                                 double alpha) {
    geometry_msgs::msg::Vector3 filtered;
    filtered.x = lowPassFilterScalar(lastValue.x, targetValue.x, alpha);
    filtered.y = lowPassFilterScalar(lastValue.y, targetValue.y, alpha);
    filtered.z = lowPassFilterScalar(lastValue.z, targetValue.z, alpha);
    return filtered;
}

// Helper for low-pass filtering orientation (Euler angles)
auto lowPassFilterOrientation(const decltype(nikita_interfaces::msg::Pose::orientation)& lastValue,
                              const decltype(nikita_interfaces::msg::Pose::orientation)& targetValue,
                              double alpha) {
    decltype(nikita_interfaces::msg::Pose::orientation) filtered;
    filtered.roll = lowPassFilterScalar(lastValue.roll, targetValue.roll, alpha);
    filtered.pitch = lowPassFilterScalar(lastValue.pitch, targetValue.pitch, alpha);
    filtered.yaw = lowPassFilterScalar(lastValue.yaw, targetValue.yaw, alpha);
    return filtered;
}

// Helper for low-pass filtering a Twist
geometry_msgs::msg::Twist lowPassFilterTwist(const geometry_msgs::msg::Twist& lastValue,
                                             const geometry_msgs::msg::Twist& targetValue, double alpha) {
    geometry_msgs::msg::Twist filtered;
    filtered.linear = lowPassFilterVector3(lastValue.linear, targetValue.linear, alpha);
    filtered.angular = lowPassFilterVector3(lastValue.angular, targetValue.angular, alpha);
    return filtered;
}

// Helper for low-pass filtering a Pose
nikita_interfaces::msg::Pose lowPassFilterPose(const nikita_interfaces::msg::Pose& lastValue,
                                                  const nikita_interfaces::msg::Pose& targetValue,
                                                  double alpha) {
    nikita_interfaces::msg::Pose filtered;
    filtered.position = lowPassFilterVector3(lastValue.position, targetValue.position, alpha);
    filtered.orientation = lowPassFilterOrientation(lastValue.orientation, targetValue.orientation, alpha);
    return filtered;
}
