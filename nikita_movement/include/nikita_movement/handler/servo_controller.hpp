/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "handler/offline_servo_protocol.hpp"
#include "nikita_interfaces/msg/servo_angle.hpp"
#include "nikita_interfaces/msg/servo_angles.hpp"
#include "nikita_interfaces/msg/servo_direct_request.hpp"
#include "nikita_interfaces/msg/servo_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "servo_protocol.hpp"
#include "std_msgs/msg/header.hpp"

class CServo {
   public:
    CServo(const std::string name, uint8_t serial_id, bool orientation_clockwise, double offset_degree,
           double adaptation)
        : name_(name),
          serial_id_(serial_id),
          orientation_clockwise_(orientation_clockwise),
          offset_degree_(offset_degree),
          adaptation_(adaptation) {
    }

    CServo() = default;
    virtual ~CServo() = default;

    std::string getName() const {
        return name_;
    }
    int getSerialID() const {
        return serial_id_;
    }
    bool isOrientationClockwise() const {
        return orientation_clockwise_;
    }
    double getOffsetDegree() const {
        return offset_degree_;
    }
    double getAdaptation() const {
        return adaptation_;
    }
    double getVoltage() const {
        return voltage_;
    }
    double getAngle() const {
        return angle_;
    }
    int getErrorCode() const {
        return error_code_;
    }
    double getTemperature() const {
        return temperature_;
    }
    void setVoltage(double voltage) {
        voltage_ = voltage;
    }
    void setAngle(double angle) {
        angle_ = angle;
    }
    void setErrorCode(int error_code) {
        error_code_ = error_code;
    }
    void setTemperature(double temperature) {
        temperature_ = temperature;
    }
    void setAdaptation(double adaptation) {
        adaptation_ = adaptation;
    }
    void setOffsetDegree(double offset_degree) {
        offset_degree_ = offset_degree;
    }
    void setOrientationClockwise(bool orientation_clockwise) {
        orientation_clockwise_ = orientation_clockwise;
    }
    void setSerialID(int serial_id) {
        serial_id_ = serial_id;
    }
    void setName(const std::string& name) {
        name_ = name;
    }

   private:
    std::string name_;
    uint8_t serial_id_;
    bool orientation_clockwise_;
    double offset_degree_;
    double adaptation_;
    double voltage_ = 12.0;
    double angle_ = 0.0;
    int error_code_ = nikita_interfaces::msg::ServoStatus::NO_ERROR;
    double temperature_ = 24.0;
};

class CServoController {
   public:
    CServoController(std::shared_ptr<rclcpp::Node> node);
    virtual ~CServoController() = default;

    void requestAngles(const std::map<uint32_t, double>& targetAngles, const double duration_s);
    // void sendServoRequest(const nikita_interfaces::msg::ServoRequest& msg);

    // void onServoRequestReceived(const nikita_interfaces::msg::ServoRequest& msg);
    void onSingleServoRequestReceived(const nikita_interfaces::msg::ServoAngle& msg);
    void onServoDirectRequestReceived(const nikita_interfaces::msg::ServoDirectRequest& msg);
    void triggerConnection();

   private:
    void onTimerStatus();

    double ticks_to_angle(int ticks, int idx);
    int angle_to_ticks(double angle, int idx);

    std::map<int, CServo> servos_;
    std::map<std::string, int> nameToIdx_;
    uint8_t cycleCounter_ = 0;
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<nikita_interfaces::msg::ServoAngle>::SharedPtr subSingleServoRequest_;
    rclcpp::Subscription<nikita_interfaces::msg::ServoDirectRequest>::SharedPtr subServoDirectRequest_;
    rclcpp::Publisher<nikita_interfaces::msg::ServoStatus>::SharedPtr pubStatus_;
    rclcpp::Publisher<nikita_interfaces::msg::ServoAngles>::SharedPtr pubAngles_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time actualTime_;
    rclcpp::Time lastTime_;

    std::shared_ptr<CServoProtocol> protocol_;
};
