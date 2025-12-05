/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/error_management.hpp"

using namespace nikita_interfaces::msg;

namespace brain {

CErrorManagement::CErrorManagement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    node_->declare_parameter("supply_voltage", rclcpp::PARAMETER_DOUBLE);
    ksupply_voltage_ = node_->get_parameter("supply_voltage").get_parameter_value().get<std::float_t>();
    node_->declare_parameter("supply_voltage_low", rclcpp::PARAMETER_DOUBLE);
    ksupply_voltage_low_ =
        node_->get_parameter("supply_voltage_low").get_parameter_value().get<std::float_t>();
    node_->declare_parameter("supply_voltage_critical_low", rclcpp::PARAMETER_DOUBLE);
    ksupply_voltage_critical_low_ =
        node_->get_parameter("supply_voltage_critical_low").get_parameter_value().get<std::float_t>();

    node_->declare_parameter("servo_voltage", rclcpp::PARAMETER_DOUBLE);
    kservo_voltage_ = node_->get_parameter("servo_voltage").get_parameter_value().get<std::float_t>();
    node_->declare_parameter("servo_voltage_low", rclcpp::PARAMETER_DOUBLE);
    kservo_voltage_low_ = node_->get_parameter("servo_voltage_low").get_parameter_value().get<std::float_t>();
    node_->declare_parameter("servo_voltage_critical_low", rclcpp::PARAMETER_DOUBLE);
    kservo_voltage_critical_low_ =
        node_->get_parameter("servo_voltage_critical_low").get_parameter_value().get<std::float_t>();

    node->declare_parameter("servo_temperature_high", rclcpp::PARAMETER_DOUBLE);
    kservo_temperature_high_ =
        node->get_parameter("servo_temperature_high").get_parameter_value().get<std::float_t>();

    node->declare_parameter("servo_temperature_critical_high", rclcpp::PARAMETER_DOUBLE);
    kservo_temperature_critical_high_ =
        node->get_parameter("servo_temperature_critical_high").get_parameter_value().get<std::float_t>();

    supplyVoltageFiltered_ = ksupply_voltage_;
    servoVoltageFiltered_ = kservo_voltage_;

    for (int i = 0; i < 10; i++) {
        filterForSupplyVoltage_.push_back(ksupply_voltage_);
        filterForServoVoltage_.push_back(kservo_voltage_);
    }
}

EError CErrorManagement::getErrorServo(const ServoStatus& msg) {
    EError error = getStatusServoTemperature(msg);
    if (error != EError::None) {
        return error;
    }
    error = filterServoVoltage(msg);
    if (error != EError::None) {
        return error;
    }
    return EError::None;
}

// private methods:
EError CErrorManagement::getStatusServoTemperature(const ServoStatus& msg) {
    if (msg.max_temperature > kservo_temperature_critical_high_) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Servo temperature of "
                                                     << msg.servo_max_temperature << " is critical: "
                                                     << to_string_with_precision(msg.max_temperature, 0)
                                                     << "°C");

        return EError::TemperatureCriticalHigh;
    }
    if (msg.max_temperature > kservo_temperature_high_) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "Servo temperature of "
                               << msg.servo_max_temperature
                               << " is high: " << to_string_with_precision(msg.max_temperature, 0) << "°C");
        return EError::TemperatureHigh;
    }
    return EError::None;
}

EError CErrorManagement::getStatusVoltage(const float voltage, const float voltageLow,
                                          const float voltageCriticalLow) {
    if (voltage < voltageCriticalLow) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "Servo voltage is critical low: "
                                << to_string_with_precision(servoVoltageFiltered_, 1) + " Volt");
        return EError::VoltageCriticalLow;
    }
    if (voltage < voltageLow) {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Servo voltage is low: " << to_string_with_precision(servoVoltageFiltered_, 1) + " Volt");
        return EError::VoltageLow;
    }
    return EError::None;
}

float CErrorManagement::calculateAverageValue(std::deque<float>& filteredValues, float voltage) {
    filteredValues.pop_front();
    filteredValues.push_back(voltage);

    float averageValue = 0;
    std::for_each(filteredValues.begin(), filteredValues.end(),
                  [&averageValue](float const& value) { averageValue += value; });
    averageValue /= filteredValues.size();
    return averageValue;
}

EError CErrorManagement::filterSupplyVoltage(float voltage) {
    auto supplyVoltageFiltered_ = calculateAverageValue(filterForSupplyVoltage_, voltage);
    return getStatusVoltage(supplyVoltageFiltered_, ksupply_voltage_low_, ksupply_voltage_critical_low_);
}

EError CErrorManagement::filterServoVoltage(const ServoStatus& msg) {
    auto servoVoltageFiltered_ = calculateAverageValue(filterForServoVoltage_, msg.max_voltage);
    return getStatusVoltage(servoVoltageFiltered_, kservo_voltage_low_, kservo_voltage_critical_low_);
}

float CErrorManagement::getFilteredSupplyVoltage() {
    return supplyVoltageFiltered_;
}

float CErrorManagement::getFilteredServoVoltage() {
    return servoVoltageFiltered_;
}

}  //namespace brain
