/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <cmath>
#include <geometry_msgs/msg/twist.hpp>

namespace utils {

constexpr double EPS = 1e-6;

bool isZero(const geometry_msgs::msg::Twist& t) {
    if (std::abs(t.linear.x) > EPS) {
        return false;
    }
    if (std::abs(t.linear.y) > EPS) {
        return false;
    }
    if (std::abs(t.linear.z) > EPS) {
        return false;
    }
    if (std::abs(t.angular.x) > EPS) {
        return false;
    }
    if (std::abs(t.angular.y) > EPS) {
        return false;
    }
    if (std::abs(t.angular.z) > EPS) {
        return false;
    }
    return true;
}

bool hasChanged(const geometry_msgs::msg::Twist& t1, const geometry_msgs::msg::Twist& t2,
                double threshold = EPS) {
    if (std::abs(t1.linear.x - t2.linear.x) > threshold) {
        return true;
    }
    if (std::abs(t1.linear.y - t2.linear.y) > threshold) {
        return true;
    }
    if (std::abs(t1.linear.z - t2.linear.z) > threshold) {
        return true;
    }
    if (std::abs(t1.angular.x - t2.angular.x) > threshold) {
        return true;
    }
    if (std::abs(t1.angular.y - t2.angular.y) > threshold) {
        return true;
    }
    if (std::abs(t1.angular.z - t2.angular.z) > threshold) {
        return true;
    }
    return false;
}

double linearMagnitude(const geometry_msgs::msg::Twist& t) {
    return std::sqrt(t.linear.x * t.linear.x + t.linear.y * t.linear.y + t.linear.z * t.linear.z);
}

double angularMagnitude(const geometry_msgs::msg::Twist& t) {
    return std::sqrt(t.angular.x * t.angular.x + t.angular.y * t.angular.y + t.angular.z * t.angular.z);
}

}  // namespace utils

using utils::angularMagnitude;
using utils::hasChanged;
using utils::isZero;
using utils::linearMagnitude;