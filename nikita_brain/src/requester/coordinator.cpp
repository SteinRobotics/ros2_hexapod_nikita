/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/coordinator.hpp"

#include <sstream>

using namespace nikita_interfaces::msg;
using namespace std::chrono_literals;

namespace brain {

CCoordinator::CCoordinator(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionPlanner> actionPlanner)
    : node_(node), actionPlanner_(actionPlanner) {
    textInterpreter_ = std::make_shared<CTextInterpreter>(node_);
    errorManagement_ = std::make_shared<CErrorManagement>(node_);

    param_velocity_factor_linear_ = node->declare_parameter<std::float_t>("velocity_factor_linear");
    param_velocity_factor_rotation_ = node->declare_parameter<std::float_t>("velocity_factor_rotation");
    param_body_factor_height_ = node->declare_parameter<std::float_t>("body_factor_height");
    param_joystick_deadzone_ = node->declare_parameter<std::float_t>("joystick_deadzone");

    if (node->declare_parameter<bool>("autostart_listening")) {
        submitSingleRequest<RequestListening>(Prio::Background, true);
    }

    param_activate_movement_waiting_ = node->declare_parameter<bool>("activate_movement_waiting");
    timerMovementRequest_ = std::make_shared<CSimpleTimer>(false);
    timerErrorRequest_ = std::make_shared<CSimpleTimer>();
    timerNoRequest_ = std::make_shared<CSimpleTimer>(param_activate_movement_waiting_);
    timerLeaveMove_ = std::make_shared<CSimpleTimer>();
}

void CCoordinator::joystickRequestReceived(const JoystickRequest& msg) {
    // BUTTON SELECT
    if (msg.button_select) {
        requestShutdown(Prio::High);
        return;
    }

    double duration_s = 0.0;
    std::string comment = "";
    uint32_t newMovementType = MovementRequest::NO_REQUEST;
    nikita_interfaces::msg::Pose body;

    // BUTTONS
    if (msg.button_a) {
        newMovementType = MovementRequest::WATCH;
        duration_s = 5.0;
    } else if (msg.button_b) {
        newMovementType = MovementRequest::HIGH_FIVE;
        comment = "ich gebe dir ein High Five";
        duration_s = 5.0;
    } else if (msg.button_x) {
        // stop listening
    } else if (msg.button_y) {
        // start listening
    } else if (msg.button_l1) {
        newMovementType = MovementRequest::LOOK_LEFT;
        duration_s = 3.0;
    } else if (msg.button_l2) {
    } else if (msg.button_r1) {
        newMovementType = MovementRequest::LOOK_RIGHT;
        duration_s = 3.0;
    } else if (msg.button_r2) {
    } else if (msg.button_start) {
        newMovementType = MovementRequest::TRANSPORT;
        duration_s = 3.0;
        comment = "ich mache mich bereit für den Transport";
    }

    // DPAD
    // int8 dpad_vertical     # DOWN = -1, UP = 1
    // int8 dpad_horizontal   # LEFT = -1, RIGHT = 1
    if (msg.dpad_vertical == 1 && !isStanding_) {
        newMovementType = MovementRequest::STAND_UP;
        duration_s = 3.0;
        comment = "ich stehe auf";
    } else if (msg.dpad_vertical == -1 && isStanding_) {
        newMovementType = MovementRequest::LAYDOWN;
        duration_s = 3.0;
        comment = "ich lege mich hin";
    }

    // LEFT_STICK -> linear movement
    // float32 left_stick_vertical   # TOP  = -1.0, DOWN = 1.0,  hangs on 0.004 -> means 0.0
    auto velocity = geometry_msgs::msg::Twist();
    if (msg.left_stick_vertical > param_joystick_deadzone_ ||
        msg.left_stick_vertical < -param_joystick_deadzone_) {
        velocity.linear.x = msg.left_stick_vertical * param_velocity_factor_linear_;
        newMovementType = MovementRequest::MOVE;
    }
    // float32 left_stick_horizontal # LEFT = -1.0, RIGHT = 1.0, hangs on 0.004 -> means 0.0
    if (msg.left_stick_horizontal > param_joystick_deadzone_ ||
        msg.left_stick_horizontal < -param_joystick_deadzone_) {
        velocity.linear.y = msg.left_stick_horizontal * param_velocity_factor_linear_;
        newMovementType = MovementRequest::MOVE;
    }
    // RIGHT_STICK -> rotation
    // float32 right_stick_horizontal  # LEFT = -1.0, RIGHT = 1.0, hangs on 0.004 -> means 0.0
    if (msg.right_stick_horizontal > param_joystick_deadzone_ ||
        msg.right_stick_horizontal < -param_joystick_deadzone_) {
        velocity.angular.z = msg.right_stick_horizontal * param_velocity_factor_rotation_;
        newMovementType = MovementRequest::MOVE;
    }

    // float32 right_stick_vertical    # TOP  = -1.0, DOWN = 1.0, hangs on 0.004 -> means 0.0
    if (msg.right_stick_vertical > param_joystick_deadzone_ ||
        msg.right_stick_vertical < -param_joystick_deadzone_) {
        body.position.z = msg.right_stick_vertical * param_body_factor_height_;
        newMovementType = MovementRequest::MOVE;
    } else {
        body.position.z = 0.0;
    }

    if (isNewMoveRequestLocked_ && actualMovementType_ == newMovementType) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "isNewMoveRequestLocked is true, ignoring joystick request");
        return;
    }

    if (actualMovementType_ == MovementRequest::MOVE && newMovementType == MovementRequest::NO_REQUEST &&
        !timerLeaveMove_->isRunning()) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "end move request");
        timerLeaveMove_->waitSecondsNonBlocking(3.0, std::bind(&CCoordinator::callbackLeaveMove, this));
        // submit zero velocity request
        submitSingleRequest<CRequestMoveVelocity>(Prio::High, velocity);
        return;
    }
    // cancel leave move
    if (newMovementType == MovementRequest::MOVE && timerLeaveMove_->isRunning()) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "cancel leave move");
        timerLeaveMove_->stop();
    }

    if (newMovementType == MovementRequest::NO_REQUEST) {
        return;
    }
    submitRequestMove(newMovementType, duration_s, body, velocity, comment, Prio::High);
}

void CCoordinator::callbackLeaveMove() {
    submitRequestMove(MovementRequest::MOVE_TO_STAND, 1.0, "", Prio::High);
}

void CCoordinator::speechRecognized(std::string text) {
    auto identifiedWords = textInterpreter_->parseText(text);
    auto command = textInterpreter_->searchInterpretation(identifiedWords);
    RCLCPP_INFO_STREAM(node_->get_logger(), "next command is: " << command);

    // TODO change this to a function map

    if (command == "notFound") {
        requestNotFound(text);
    } else if (command == "commandStandup") {
        submitRequestMove(MovementRequest::STAND_UP, 3.0, "ich stehe auf", Prio::High);
    } else if (command == "commandLaydown") {
        submitRequestMove(MovementRequest::LAYDOWN, 3.0, "ich leg mich hin", Prio::High);
    } else if (command == "commandWatch") {
        submitRequestMove(MovementRequest::WATCH, 5.0, "ich schaue mich um", Prio::High);
    } else if (command == "commandTurnHead") {
        RCLCPP_INFO_STREAM(node_->get_logger(), "identifiedWords: " << identifiedWords.size());
        for (size_t i = 0; i < identifiedWords.size(); ++i) {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "identifiedWords[" << i << "]: " << identifiedWords[i].value);
        }

        if (textInterpreter_->lettersIdentified("links", identifiedWords)) {
            submitRequestMove(MovementRequest::LOOK_LEFT, 3.0, "ich schaue nach links", Prio::High);
        } else if (textInterpreter_->lettersIdentified("rechts", identifiedWords)) {
            submitRequestMove(MovementRequest::LOOK_RIGHT, 3.0, "ich schaue nach rechts", Prio::High);
        }
    } else if (command == "commandTransport") {
        submitRequestMove(MovementRequest::TRANSPORT, 5.0, "ich bereite mich auf den Transport vor",
                          Prio::High);
    } else if (command == "commandMove") {
        // requestMoveBody(detectedWords);
        // requestStopMoveBody();
    } else if (command == "commandTestBody") {
        requestTestBody();
    } else if (command == "commandTestLegs") {
        requestTestLegs();
    } else if (command == "talk") {
        // requestTalking(awareness_->m_speechRecognized);
    } else if (command == "chat") {
        // requestChat(awareness_->m_speechRecognized);
    } else if (command == "tellMeSupplyVoltage") {
        requestTellSupplyVoltage(Prio::High);
    } else if (command == "tellMeServoVoltage") {
        requestTellServoVoltage(Prio::High);
    } else if (command == "tellMeServoTemperature") {
        requestTellServoTemperature(Prio::High);
    } else if (command == "musicOn") {
        requestMusikOn();
    } else if (command == "musicOff") {
        requestMusikOff();
    }
}

void CCoordinator::supplyVoltageReceived(float voltage) {
    auto statusSupplyVoltage = errorManagement_->filterSupplyVoltage(voltage);
    if (statusSupplyVoltage == EError::VoltageCriticalLow) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "SupplyVoltage is critical low, Shutting down the system");

        std::string text = "Versorgungsspannung extrem niedrig, schalte ab";
        requestReactionOnError(text, true, Prio::Highest);

    } else if (statusSupplyVoltage == EError::VoltageLow) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "SupplyVoltage is low");
        std::string text = "Versorgungsspannung ist niedrig bei " +
                           to_string_with_precision(errorManagement_->getFilteredSupplyVoltage(), 1) +
                           " Volt";
        requestReactionOnError(text, false);
    }
}

void CCoordinator::servoStatusReceived(const ServoStatus& msg) {
    std::string text = "";
    auto error = errorManagement_->getErrorServo(msg);
    if (error == EError::None) {
        return;
    }
    if (error == EError::VoltageCriticalLow) {
        text = "Servo Spannung ist niedrig";
        requestReactionOnError(text, false, Prio::Highest);
    } else {
        text = "Servo Fehler " + errorManagement_->getErrorName(error);
        requestReactionOnError(text, false, Prio::High);
    }
}

// ----------------------------------------------------------------------------
//  Requests
// ---------------------------------------------------------------------------
template <typename RequestT, typename... Args>
void CCoordinator::submitSingleRequest(Prio prio, Args&&... args) {
    static_assert(std::is_base_of<RequestBase, RequestT>::value, "RequestT must derive from RequestBase");
    std::vector<std::shared_ptr<RequestBase>> request_v;
    request_v.push_back(std::make_shared<RequestT>(std::forward<Args>(args)...));
    actionPlanner_->request(request_v, prio);
}

void CCoordinator::requestNotFound(std::string textRecognized, Prio prio) {
    std::string textOutput = "Ich habe folgendes nicht verstanden " + textRecognized;
    submitSingleRequest<RequestTalking>(prio, textOutput);
}

void CCoordinator::requestShutdown(Prio prio) {
    std::string text = "Ich muss mich jetzt abschalten";
    submitSingleRequest<RequestTalking>(prio, text);

    if (isStanding_) {
        submitRequestMove(MovementRequest::LAYDOWN, 2.0, "", prio);
    }
    submitSingleRequest<RequestSystem>(prio, true, true);
}

void CCoordinator::requestReactionOnError(std::string text, bool isShutdownRequested, Prio prio) {
    if (timerErrorRequest_->isRunning()) {
        return;
    }
    // timerErrorRequest_->startWithoutCallback(10s);

    submitSingleRequest<RequestTalking>(prio, text);

    if (isShutdownRequested) {
        submitRequestMove(MovementRequest::LAYDOWN, 3.0, "", prio);
    }
    submitSingleRequest<RequestSystem>(prio, isShutdownRequested, isShutdownRequested);
}

void CCoordinator::requestMusikOn(Prio prio) {
    std::string song = "musicfox_hot_dogs_for_breakfast.mp3";
    actionPlanner_->request({std::make_shared<RequestMusic>(song)}, prio);
}

void CCoordinator::requestMusikOff(Prio prio) {
    actionPlanner_->request({std::make_shared<RequestListening>(false)}, prio);
}

void CCoordinator::requestTalking(std::string text, Prio prio) {
    actionPlanner_->request({std::make_shared<RequestTalking>(text)}, prio);
}

void CCoordinator::requestChat(std::string text, Prio prio) {
    actionPlanner_->request({std::make_shared<RequestChat>(text)}, prio);
}

void CCoordinator::requestTestBody() {
    nikita_interfaces::msg::Pose body;
    double duration_s = 2.0;
    std::string text = "";

    text = "teste x Richtung";
    body.position.x = 0.05;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, text, Prio::High);
    body.position.x = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, "", Prio::High);

    text = "teste y Richtung";
    body.position.y = 0.05;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, text, Prio::High);
    body.position.y = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, "", Prio::High);

    text = "teste z Richtung nach oben";
    body.position.z = 0.03;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, text, Prio::High);
    body.position.z = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, "", Prio::High);

    text = "teste Roll";
    body.orientation.roll = 20.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, text, Prio::High);
    body.orientation.roll = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, "", Prio::High);

    text = "teste Pitch";
    body.orientation.pitch = 20.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, text, Prio::High);
    body.orientation.pitch = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, "", Prio::High);

    text = "teste Yaw";
    body.orientation.yaw = 20.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, text, Prio::High);
    body.orientation.yaw = 0.0;
    submitRequestMove(MovementRequest::TESTBODY, duration_s, body, "", Prio::High);
}

void CCoordinator::requestTestLegs() {
    submitRequestMove(MovementRequest::TESTLEGS, 2.0, "", Prio::High);
}

void CCoordinator::requestWaiting(Prio prio) {
    actualMovementType_ = MovementRequest::WAITING;
    submitRequestMove(actualMovementType_, 5.0, "ich warte", prio);
}

void CCoordinator::submitRequestMove(uint32_t movementType, double duration_s,
                                     std::optional<geometry_msgs::msg::Twist> velocity, std::string comment,
                                     Prio prio) {
    submitRequestMove(movementType, duration_s, std::nullopt, velocity, comment, prio);
}

void CCoordinator::submitRequestMove(uint32_t movementType, double duration_s,
                                     std::optional<nikita_interfaces::msg::Pose> body, std::string comment,
                                     Prio prio) {
    submitRequestMove(movementType, duration_s, body, std::nullopt, comment, prio);
}

void CCoordinator::submitRequestMove(uint32_t movementType, double duration_s, std::string comment,
                                     Prio prio) {
    submitRequestMove(movementType, duration_s, std::nullopt, std::nullopt, comment, prio);
}

void CCoordinator::submitRequestMove(uint32_t movementType, double duration_s,
                                     std::optional<nikita_interfaces::msg::Pose> body,
                                     std::optional<geometry_msgs::msg::Twist> velocity, std::string comment,
                                     Prio prio) {
    std::vector<std::shared_ptr<RequestBase>> request_v;
    if (!comment.empty()) {
        request_v.push_back(std::make_shared<RequestTalking>(comment));
    }
    // If we are not standing, we need to stand up first
    if (!isStanding_ && movementType == MovementRequest::MOVE) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "standup before move request");
        isStanding_ = true;
        // recursive call to first stand up
        submitRequestMove(MovementRequest::STAND_UP, 3.0, "ich stehe erst mal auf", prio);
    }

    auto request = MovementRequest();
    request.type = movementType;
    request.duration_s = duration_s;
    request.name = movementTypeName_.at(request.type);
    request_v.push_back(std::make_shared<CRequestMovementType>(request));
    if (body.has_value()) {
        request_v.push_back(std::make_shared<CRequestMoveBody>(body.value()));
    }
    if (velocity.has_value()) {
        request_v.push_back(std::make_shared<CRequestMoveVelocity>(velocity.value()));
    }
    actionPlanner_->request(request_v, prio);

    // update the actual movement type, it should only be updated in this function!
    actualMovementType_ = movementType;

    // update the isStanding_ state, it should only be updated in this function!
    if (movementType == MovementRequest::LAYDOWN || movementType == MovementRequest::TRANSPORT) {
        isStanding_ = false;
    } else if (movementType == MovementRequest::STAND_UP) {
        isStanding_ = true;
    }

    // Lock the new move request for the given duration except for MOVE requests
    if (MovementRequest::MOVE == movementType) {
        return;
    }
    isNewMoveRequestLocked_ = true;
    // RCLCPP_INFO_STREAM(node_->get_logger(), "isNewMoveRequestLocked_ locked");
    timerMovementRequest_->waitSecondsNonBlocking(duration_s, [this]() {
        isNewMoveRequestLocked_ = false;
        actualMovementType_ = MovementRequest::NO_REQUEST;
        // RCLCPP_INFO_STREAM(node_->get_logger(), "isNewMoveRequestLocked_ released");
    });
}

void CCoordinator::requestTellSupplyVoltage(Prio prio) {
    std::string text = "Die Versorgungsspannung ist aktuell " +
                       to_string_with_precision(errorManagement_->getFilteredSupplyVoltage(), 1) + " Volt";
    submitSingleRequest<RequestTalking>(prio, text);
}

void CCoordinator::requestTellServoVoltage(Prio prio) {
    std::string text = "Die Servo Spannung ist aktuell " +
                       to_string_with_precision(errorManagement_->getFilteredServoVoltage(), 1) + " Volt";
    submitSingleRequest<RequestTalking>(prio, text);
}

void CCoordinator::requestTellServoTemperature(Prio prio) {
    std::string text = "Die Servo Spannung ist aktuell " +
                       to_string_with_precision(errorManagement_->getFilteredServoVoltage(), 1) + " Volt";
    submitSingleRequest<RequestTalking>(prio, text);
}

// ----------------------------------------------------------------------------
//  update
// ---------------------------------------------------------------------------
void CCoordinator::update() {
    if (param_activate_movement_waiting_ && actualMovementType_ == MovementRequest::NO_REQUEST) {
        if (!timerNoRequest_->isRunning()) {
            timerNoRequest_->start();
        }

        // If no request is received for 30 seconds, we request a default move
        if (timerNoRequest_->haveSecondsElapsed(30.0)) {
            if (isNewMoveRequestLocked_) {
                RCLCPP_WARN_STREAM(node_->get_logger(),
                                   "No request received for 30 seconds, but isNewMoveRequestLocked_ is true");
                return;
            }
            RCLCPP_INFO_STREAM(node_->get_logger(), "No request received for 30 seconds, requesting default");
            // TODO: change all other Prios to High
            // requestWaiting(Prio::Normal);
        }

    } else {
        timerNoRequest_->stop();
    }
}

}  // namespace brain
