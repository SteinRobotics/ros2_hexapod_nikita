/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#pragma once

#include <deque>

#include "nikita_interfaces/msg/servo_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/utility.hpp"

namespace brain {

enum class EError {
    None = 0,
    VoltageLow = 1,
    VoltageCriticalLow = 2,
    VoltageHigh = 3,
    TemperatureHigh = 4,
    TemperatureCriticalHigh = 5,
};

class CErrorManagement {
   public:
    CErrorManagement(std::shared_ptr<rclcpp::Node> node);
    virtual ~CErrorManagement() = default;

    EError getErrorServo(const nikita_interfaces::msg::ServoStatus& msg);
    EError filterSupplyVoltage(float voltage);

    float getFilteredSupplyVoltage();
    float getFilteredServoVoltage();

    std::string getErrorName(EError error) {
        switch (error) {
            case EError::None:
                return "none";
            case EError::VoltageLow:
                return "voltage low";
            case EError::VoltageCriticalLow:
                return "voltage critical low";
            case EError::VoltageHigh:
                return "voltage high";
            case EError::TemperatureHigh:
                return "temperature high";
            case EError::TemperatureCriticalHigh:
                return "temperature critical high";
            default:
                return "unknown error";
        }
    }

   private:
    EError filterServoVoltage(const nikita_interfaces::msg::ServoStatus& msg);
    EError getStatusServoTemperature(const nikita_interfaces::msg::ServoStatus& msg);
    EError getStatusVoltage(const float voltage, const float voltageLow, const float voltageCriticalLow);
    float calculateAverageValue(std::deque<float>& filteredValues, float voltage);

    std::shared_ptr<rclcpp::Node> node_;

    float ksupply_voltage_ = 0.0;
    float ksupply_voltage_low_ = 0.0;
    float ksupply_voltage_critical_low_ = 0.0;

    float kservo_voltage_ = 0.0;
    float kservo_voltage_low_ = 0.0;
    float kservo_voltage_critical_low_ = 0.0;

    float kservo_temperature_high_ = 0.0;
    float kservo_temperature_critical_high_ = 0.0;

    float supplyVoltageFiltered_ = 0.0;
    std::deque<float> filterForSupplyVoltage_;
    float servoVoltageFiltered_ = 0.0;
    std::deque<float> filterForServoVoltage_;
};

}  // namespace brain
