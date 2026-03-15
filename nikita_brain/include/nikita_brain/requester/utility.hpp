/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <format>
#include <map>
#include <string>

#include "nikita_interfaces/msg/movement_request.hpp"

namespace brain {

const std::map<const uint32_t, const std::string> movementTypeToName = {
    {nikita_interfaces::msg::MovementRequest::NO_REQUEST, "NO_REQUEST"},
    {nikita_interfaces::msg::MovementRequest::LAYDOWN, "LAYDOWN"},
    {nikita_interfaces::msg::MovementRequest::STAND_UP, "STAND_UP"},
    {nikita_interfaces::msg::MovementRequest::WAITING, "WAITING"},
    {nikita_interfaces::msg::MovementRequest::MOVE_TRIPOD, "MOVE_TRIPOD"},
    {nikita_interfaces::msg::MovementRequest::MOVE_RIPPLE, "MOVE_RIPPLE"},
    {nikita_interfaces::msg::MovementRequest::MOVE_WAVE, "MOVE_WAVE"},
    {nikita_interfaces::msg::MovementRequest::WATCH, "WATCH"},
    {nikita_interfaces::msg::MovementRequest::LOOK, "LOOK"},
    {nikita_interfaces::msg::MovementRequest::DANCE, "DANCE"},
    {nikita_interfaces::msg::MovementRequest::HIGH_FIVE, "HIGH_FIVE"},
    {nikita_interfaces::msg::MovementRequest::LEGS_WAVE, "LEGS_WAVE"},
    {nikita_interfaces::msg::MovementRequest::BODY_ROLL, "BODY_ROLL"},
    {nikita_interfaces::msg::MovementRequest::BITE, "BITE"},
    {nikita_interfaces::msg::MovementRequest::STOMP, "STOMP"},
    {nikita_interfaces::msg::MovementRequest::CLAP, "CLAP"},
    {nikita_interfaces::msg::MovementRequest::CONTINUOUS_POSE, "CONTINUOUS_POSE"},
    {nikita_interfaces::msg::MovementRequest::SINGLE_POSE, "SINGLE_POSE"},
    {nikita_interfaces::msg::MovementRequest::TESTLEGS, "TESTLEGS"},
    {nikita_interfaces::msg::MovementRequest::NEUTRAL, "NEUTRAL"},
    {nikita_interfaces::msg::MovementRequest::CALIBRATE, "CALIBRATE"},
    {nikita_interfaces::msg::MovementRequest::MOVE, "MOVE"},
};

// Auto-generated reverse map from movementTypeToName
inline const auto nameToMovementType = [] {
    std::map<const std::string, uint32_t> result;
    for (const auto& [key, val] : movementTypeToName) {
        result[val] = key;
    }
    return result;
}();

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2) {
    return std::format("{:.{}f}", a_value, n);
}
}  // namespace brain