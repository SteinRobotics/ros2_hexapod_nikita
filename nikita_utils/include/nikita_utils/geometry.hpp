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

inline bool isCosValueNearZero(double angle, double angle_threshold) {
    auto output_lower = std::cos(angle);
    auto output_upper = std::cos(angle + angle_threshold);

    int32_t sign_output_lower = (output_lower >= 0.0) ? 1 : -1;
    int32_t sign_output_upper = (output_upper >= 0.0) ? 1 : -1;

    return sign_output_lower != sign_output_upper;
}

// M_PI_4 means that sin and cos values are equal at that point
inline bool areSinCosValuesEqual(double angle_rad, double angle_rad_threshold) {
    auto angle_modulo = std::fmod(std::abs(angle_rad), M_PI_4);
    auto diff = angle_modulo - M_PI_4;
    return std::abs(diff) <= angle_rad_threshold;
}

}  // namespace utils

using utils::deg2rad;
using utils::rad2deg;
using utils::TWO_PI;