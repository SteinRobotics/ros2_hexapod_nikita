/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>

namespace utils {

// limitation filter
template <typename T>
static T filterChangeRate(const T& lastValue, const T& targetValue, const T& changeRate) {
    return std::clamp(targetValue, lastValue - changeRate, lastValue + changeRate);
}

// low-pass filter
template <typename T>
static T lowPassFilter(const T& lastValue, const T& targetValue, double alpha) {
    return alpha * targetValue + (1.0 - alpha) * lastValue;
}

}  // namespace utils
