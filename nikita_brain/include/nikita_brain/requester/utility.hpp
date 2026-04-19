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
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_LAYDOWN, "SEQUENCE_LAYDOWN"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_STAND_UP, "SEQUENCE_STAND_UP"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_WAITING, "SEQUENCE_WAITING"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_WATCH, "SEQUENCE_WATCH"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_LOOK, "SEQUENCE_LOOK"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_DANCE, "SEQUENCE_DANCE"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_HIGH_FIVE, "SEQUENCE_HIGH_FIVE"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_LEGS_WAVE, "SEQUENCE_LEGS_WAVE"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_BODY_ROLL, "SEQUENCE_BODY_ROLL"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_BITE, "SEQUENCE_BITE"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_STOMP, "SEQUENCE_STOMP"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_CLAP, "SEQUENCE_CLAP"},
    {nikita_interfaces::msg::MovementRequest::CONTINUOUS_POSE, "CONTINUOUS_POSE"},
    {nikita_interfaces::msg::MovementRequest::SINGLE_POSE, "SINGLE_POSE"},
    {nikita_interfaces::msg::MovementRequest::SEQUENCE_TESTLEGS, "SEQUENCE_TESTLEGS"},
    {nikita_interfaces::msg::MovementRequest::CONTINUOUS_MOVE, "CONTINUOUS_MOVE"},
    {nikita_interfaces::msg::MovementRequest::CONTINUOUS_RUNNING, "CONTINUOUS_RUNNING"},
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