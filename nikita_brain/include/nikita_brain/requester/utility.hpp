/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <map>
#include <sstream>
#include <string>

#include "nikita_interfaces/msg/movement_request.hpp"

namespace brain {

const std::map<const uint32_t, const std::string> movementTypeToName = {
    {nikita_interfaces::msg::MovementRequest::NO_REQUEST, "NO_REQUEST"},
    {nikita_interfaces::msg::MovementRequest::LAYDOWN, "LAYDOWN"},
    {nikita_interfaces::msg::MovementRequest::STAND_UP, "STAND_UP"},
    {nikita_interfaces::msg::MovementRequest::WAITING, "WAITING"},
    {nikita_interfaces::msg::MovementRequest::MOVE_TRIPOD, "MOVE_TRIPOD"},
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
};

// Static map of movement type names to their enum values
const std::map<const std::string, const uint32_t> nameToMovementType = {
    {"NO_REQUEST", nikita_interfaces::msg::MovementRequest::NO_REQUEST},
    {"LAYDOWN", nikita_interfaces::msg::MovementRequest::LAYDOWN},
    {"STAND_UP", nikita_interfaces::msg::MovementRequest::STAND_UP},
    {"WAITING", nikita_interfaces::msg::MovementRequest::WAITING},
    {"MOVE_TRIPOD", nikita_interfaces::msg::MovementRequest::MOVE_TRIPOD},
    {"WATCH", nikita_interfaces::msg::MovementRequest::WATCH},
    {"LOOK", nikita_interfaces::msg::MovementRequest::LOOK},
    {"DANCE", nikita_interfaces::msg::MovementRequest::DANCE},
    {"HIGH_FIVE", nikita_interfaces::msg::MovementRequest::HIGH_FIVE},
    {"LEGS_WAVE", nikita_interfaces::msg::MovementRequest::LEGS_WAVE},
    {"BODY_ROLL", nikita_interfaces::msg::MovementRequest::BODY_ROLL},
    {"BITE", nikita_interfaces::msg::MovementRequest::BITE},
    {"STOMP", nikita_interfaces::msg::MovementRequest::STOMP},
    {"CLAP", nikita_interfaces::msg::MovementRequest::CLAP},
    {"CONTINUOUS_POSE", nikita_interfaces::msg::MovementRequest::CONTINUOUS_POSE},
    {"SINGLE_POSE", nikita_interfaces::msg::MovementRequest::SINGLE_POSE},
    {"TESTLEGS", nikita_interfaces::msg::MovementRequest::TESTLEGS},
    {"NEUTRAL", nikita_interfaces::msg::MovementRequest::NEUTRAL},
    {"CALIBRATE", nikita_interfaces::msg::MovementRequest::CALIBRATE},
};

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return std::move(out).str();
}
}  // namespace brain