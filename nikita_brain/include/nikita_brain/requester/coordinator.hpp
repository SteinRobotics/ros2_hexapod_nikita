/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
//
#include "nikita_interfaces/msg/joystick_request.hpp"
#include "nikita_interfaces/msg/movement_request.hpp"
#include "nikita_interfaces/msg/servo_status.hpp"
//
#include <nikita_utils/callback_timer.hpp>
#include <nikita_utils/simpletimer.hpp>

#include "action/action_planner.hpp"
#include "irequester.hpp"
#include "requester/error_management.hpp"
#include "requester/text_interpreter.hpp"
#include "requester/utility.hpp"

namespace brain {

class CCoordinator : public IRequester {
   public:
    CCoordinator(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionPlanner> jobHandler);
    virtual ~CCoordinator() = default;

    void update() override;

    void joystickRequestReceived(const nikita_interfaces::msg::JoystickRequest& msg);
    void speechRecognized(std::string text);
    void supplyVoltageReceived(float voltage);
    void servoStatusReceived(const nikita_interfaces::msg::ServoStatus& msg);

   private:
    template <typename RequestT, typename... Args>
    void submitSingleRequest(Prio prio, Args&&... args);

    void submitRequestMove(uint32_t movementType, double duration_s,
                           std::optional<geometry_msgs::msg::Twist> velocity, std::string comment = "",
                           Prio prio = Prio::Normal);
    void submitRequestMove(uint32_t movementType, double duration_s,
                           std::optional<nikita_interfaces::msg::Pose> body, std::string comment = "",
                           Prio prio = Prio::Normal);
    void submitRequestMove(uint32_t movementType, double duration_s, std::string comment = "",
                           Prio prio = Prio::Normal);
    void submitRequestMove(uint32_t movementType, double duration_s = 0.0,
                           std::optional<nikita_interfaces::msg::Pose> body = std::nullopt,
                           std::optional<geometry_msgs::msg::Twist> velocity = std::nullopt,
                           std::string comment = "", Prio prio = Prio::Normal);

    void requestShutdown(Prio prio);
    void requestReactionOnError(std::string text, bool isShutdownRequested, Prio prio = Prio::Normal);
    void requestNotFound(std::string textRecognized, Prio prio = Prio::Normal);
    void requestDefault(Prio prio = Prio::Normal);
    void requestStopMoveBody(Prio prio = Prio::Normal);
    void requestTellSupplyVoltage(Prio prio = Prio::Normal);
    void requestTellServoVoltage(Prio prio = Prio::Normal);
    void requestTellServoTemperature(Prio prio = Prio::Normal);
    void requestMusikOn(Prio prio = Prio::Normal);
    void requestMusikOff(Prio prio = Prio::Normal);
    void requestTalking(std::string text, Prio prio = Prio::Normal);
    void requestChat(std::string text, Prio prio = Prio::Normal);
    void requestWaiting(Prio prio = Prio::Normal);
    void requestTestBody();
    void requestTestLegs();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPlanner> actionPlanner_;
    std::shared_ptr<CErrorManagement> errorManagement_;
    std::shared_ptr<CTextInterpreter> textInterpreter_;

    std::shared_ptr<CSimpleTimer> timerErrorRequest_;
    std::shared_ptr<CSimpleTimer> timerNoRequest_;

    std::shared_ptr<CCallbackTimer> timerMovementRequest_;
    // std::shared_ptr<CCallbackTimer> timerLeaveMove_;

    // void callbackLeaveMove();

    std::atomic<bool> isNewMoveRequestLocked_{false};

    uint32_t actualMovementType_ = nikita_interfaces::msg::MovementRequest::NO_REQUEST;
    bool isStanding_ = false;
    bool isRelayOn_ = false;

    float param_velocity_factor_linear_ = 0.0;
    float param_velocity_factor_rotation_ = 0.0;
    float param_body_factor_height_ = 0.0;
    float param_joystick_deadzone_ = 0.0;
    float param_activate_movement_waiting_ = false;

    std::map<const uint32_t, const std::string> movementTypeName_ = {
        {nikita_interfaces::msg::MovementRequest::NO_REQUEST, "NO_REQUEST"},
        {nikita_interfaces::msg::MovementRequest::LAYDOWN, "LAYDOWN"},
        {nikita_interfaces::msg::MovementRequest::STAND_UP, "STAND_UP"},
        {nikita_interfaces::msg::MovementRequest::WAITING, "WAITING"},
        {nikita_interfaces::msg::MovementRequest::MOVE_TRIPOD, "MOVE_TRIPOD"},
        {nikita_interfaces::msg::MovementRequest::WATCH, "WATCH"},
        {nikita_interfaces::msg::MovementRequest::LOOK_LEFT, "LOOK_LEFT"},
        {nikita_interfaces::msg::MovementRequest::LOOK_RIGHT, "LOOK_RIGHT"},
        {nikita_interfaces::msg::MovementRequest::DANCE, "DANCE"},
        {nikita_interfaces::msg::MovementRequest::HIGH_FIVE, "HIGH_FIVE"},
        {nikita_interfaces::msg::MovementRequest::LEGS_WAVE, "LEGS_WAVE"},
        {nikita_interfaces::msg::MovementRequest::BODY_ROLL, "BODY_ROLL"},
        {nikita_interfaces::msg::MovementRequest::BITE, "BITE"},
        {nikita_interfaces::msg::MovementRequest::STOMP, "STOMP"},
        {nikita_interfaces::msg::MovementRequest::CLAP, "CLAP"},
        {nikita_interfaces::msg::MovementRequest::TRANSPORT, "TRANSPORT"},
        {nikita_interfaces::msg::MovementRequest::TESTBODY, "TESTBODY"},
        {nikita_interfaces::msg::MovementRequest::TESTLEGS, "TESTLEGS"},
        {nikita_interfaces::msg::MovementRequest::NEUTRAL, "NEUTRAL"},
        {nikita_interfaces::msg::MovementRequest::CALIBRATE, "CALIBRATE"},
    };
};

}  // namespace brain