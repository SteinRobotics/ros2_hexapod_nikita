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

}  // namespace utils

using utils::deg2rad;
using utils::rad2deg;
using utils::TWO_PI;