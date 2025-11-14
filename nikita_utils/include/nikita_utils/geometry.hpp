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

inline bool isSinValueNearZero(double angle, double angleThreshold) {
    auto outputLower = std::sin(angle);
    auto outputUpper = std::sin(angle + angleThreshold);

    int32_t signOutputLower = (outputLower >= 0.0) ? 1 : -1;
    int32_t signOutputUpper = (outputUpper >= 0.0) ? 1 : -1;

    return signOutputLower != signOutputUpper;
}

inline bool isCosValueNearZero(double angle, double angleThreshold) {
    auto outputLower = std::cos(angle);
    auto outputUpper = std::cos(angle + angleThreshold);

    int32_t signOutputLower = (outputLower >= 0.0) ? 1 : -1;
    int32_t signOutputUpper = (outputUpper >= 0.0) ? 1 : -1;

    return signOutputLower != signOutputUpper;
}

// M_PI_4 means that sin and cos values are equal at that point
inline bool areSinCosValuesEqual(double angle, double angleThreshold) {
    auto angleModulo = std::fmod(std::abs(angle), M_PI_4);
    auto diff = angleModulo - M_PI_4;
    return std::abs(diff) <= angleThreshold;
}

}  // namespace utils

using utils::deg2rad;
using utils::rad2deg;
using utils::TWO_PI;