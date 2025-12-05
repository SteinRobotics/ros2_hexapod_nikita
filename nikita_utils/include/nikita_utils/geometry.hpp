/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <cmath>

namespace utils {

constexpr double TWO_PI = 2.0 * M_PI;

inline double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

inline bool isSinValueNearZero(double angle_rad, double angle_rad_threshold) {
    auto angle_rad_modulo_pi = std::fmod(angle_rad, M_PI);

    if (std::abs(angle_rad_modulo_pi) <= angle_rad_threshold) {
        return true;
    }
    if (std::abs(angle_rad_modulo_pi - M_PI) <= angle_rad_threshold) {
        return true;
    }
    return false;
}

// 3/4 PI means the point where sin is equal cos and sin is decreasing
inline bool isSinNear34Pi(double angle_rad, double angle_rad_threshold) {
    if (angle_rad >= (3.0 * M_PI_4) - (angle_rad_threshold / 2.0) &&
        angle_rad < (3.0 * M_PI_4) + (angle_rad_threshold / 2.0)) {
        return true;
    }
    return false;
}

}  // namespace utils

using utils::deg2rad;
using utils::rad2deg;
using utils::TWO_PI;