/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/requester.hpp"

using namespace nikita_interfaces::msg;
using std::placeholders::_1;

CRequester::CRequester(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CServoHandler> servoHandler)
    : node_(node) {
    actionPackagesParser_ = std::make_shared<CActionPackagesParser>(node);
    kinematics_ = std::make_shared<CKinematics>(node, actionPackagesParser_);
    gaitController_ = std::make_shared<CGaitController>(node, kinematics_);
    if (servoHandler) {
        servoHandler_ = servoHandler;
    } else {
        servoHandler_ = std::make_shared<CServoHandler>(node);
    }

    initializeRequestHandlers();

    subMovementTypeRequest_ = node_->create_subscription<MovementRequest>(
        "cmd_movement_type", 10, std::bind(&CRequester::onMovementTypeRequest, this, _1));

    subMovementVelocityRequest_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&CRequester::onMovementVelocityRequest, this, _1));

    subMovementBodyPoseRequest_ = node_->create_subscription<nikita_interfaces::msg::Pose>(
        "cmd_body_pose", 10, std::bind(&CRequester::onMovementBodyPoseRequest, this, _1));
}

// ------------------------------------------------------------------------------------------------
// private methods for the CRequester class
// ------------------------------------------------------------------------------------------------------------
void CRequester::initializeRequestHandlers() {
    auto bind = [this](auto fn) { return [this, fn](const MovementRequest& msg) { (this->*fn)(msg); }; };

    requestHandlers_ = {
        {MovementRequest::NO_REQUEST,
         [this](const MovementRequest&) { activeRequest_ = MovementRequest::NO_REQUEST; }},
        {MovementRequest::LAYDOWN, bind(&CRequester::requestGait)},
        {MovementRequest::STAND_UP, bind(&CRequester::requestGait)},
        {MovementRequest::WAITING, bind(&CRequester::requestGait)},
        {MovementRequest::MOVE_TRIPOD, bind(&CRequester::requestGait)},
        {MovementRequest::WATCH, bind(&CRequester::requestGait)},
        {MovementRequest::LOOK_LEFT, bind(&CRequester::requestSequence)},
        {MovementRequest::LOOK_RIGHT, bind(&CRequester::requestSequence)},
        {MovementRequest::DANCE, bind(&CRequester::requestDance)},
        {MovementRequest::HIGH_FIVE, bind(&CRequester::requestGait)},
        {MovementRequest::LEGS_WAVE, bind(&CRequester::requestGait)},
        {MovementRequest::BODY_ROLL, bind(&CRequester::requestGait)},
        {MovementRequest::BITE, bind(&CRequester::requestBite)},
        {MovementRequest::STOMP, bind(&CRequester::requestStomp)},
        {MovementRequest::CLAP, bind(&CRequester::requestSequence)},
        {MovementRequest::TRANSPORT, bind(&CRequester::requestTransport)},
        {MovementRequest::TESTBODY, bind(&CRequester::requestTestBody)},
        {MovementRequest::TESTLEGS, bind(&CRequester::requestTestLegs)},
        {MovementRequest::NEUTRAL, bind(&CRequester::requestNeutral)},
        {MovementRequest::CALIBRATE, bind(&CRequester::requestCalibrate)},
        // Add more handlers as needed
    };
}

void CRequester::sendServoRequest(const double duration_s, const bool blocking) {
    auto head = kinematics_->getHead();
    auto legs = kinematics_->getLegsAngles();
    if (blocking) {
        servoHandler_->appendRequest(CRequest(head, legs, duration_s));
    } else {
        servoHandler_->requestWithoutQueue(CRequest(head, legs, duration_s));
    }
}

void CRequester::requestGait(const MovementRequest& msg) {
    activeRequest_ = msg.type;
    gaitController_->setGait(msg.type);
}

void CRequester::requestSequence(const MovementRequest& msg) {
    activeRequest_ = msg.type;

    auto sequence = actionPackagesParser_->getRequests(msg.name);

    for (const auto& action : sequence) {
        std::map<ELegIndex, CPosition> legPositions = kinematics_->getLegsPositions();
        CPose bodyPos = kinematics_->getBody();

        if (action.legAngles.has_value()) {
            const auto& legAnglesMap = action.legAngles.value();
            for (const auto& [legIndex, anglesData] : legAnglesMap) {
                CLegAngles legAngles(anglesData.coxa_deg, anglesData.femur_deg, anglesData.tibia_deg);
                kinematics_->setLegAngles(legIndex, legAngles);
            }
        }
        if (action.footPositions.has_value()) {
            const auto& footPositionsMap = action.footPositions.value();
            for (const auto& [legIndex, positionData] : footPositionsMap) {
                legPositions.at(legIndex) = positionData;
            }
        }
        if (action.body.has_value()) {
            auto bodyPos = CPose(action.body->position, action.body->orientation);
        }
        if (action.body.has_value() || action.footPositions.has_value()) {
            kinematics_->moveBody(legPositions, bodyPos);
        }
        if (action.head.has_value()) {
            kinematics_->setHead(action.head->yaw_deg, action.head->pitch_deg);
        }
        double duration_s = msg.duration_s * action.factorDuration;
        sendServoRequest(duration_s);
    }
}

void CRequester::requestDance(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::DANCE;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning
}

void CRequester::requestBite(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::BITE;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning
}

void CRequester::requestStomp(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::STOMP;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning
}

void CRequester::requestTransport(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::TRANSPORT;

    for (auto& [legIndex, leg] : kinematics_->getLegs()) {
        kinematics_->setLegAngles(legIndex, CLegAngles(0.0, 30.0, -20.0));
    }
    sendServoRequest(msg.duration_s);
}

void CRequester::requestNeutral(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::NEUTRAL;
    // set the head to the neutral position
    kinematics_->setHead(0.0, 0.0);
    // set all legs to the neutral position
    for (auto& [legIndex, leg] : kinematics_->getLegs()) {
        kinematics_->setLegAngles(legIndex, CLegAngles(0.0, 0.0, 0.0));
    }
    sendServoRequest(msg.duration_s);
}

void CRequester::requestCalibrate(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::CALIBRATE;

    // set all legs to the kiss ground position
    auto positionsKissGround = kinematics_->getLegsStandingPositions();
    for (auto& [legIndex, position] : positionsKissGround) {
        positionsKissGround[legIndex] = CPosition(position.x, position.y, 0.0);
    }
    kinematics_->moveBody(positionsKissGround);
    sendServoRequest(msg.duration_s);
}

void CRequester::requestTestBody(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::TESTBODY;

    // auto requestedBody = CPose(msg.body);
    kinematics_->moveBody(kinematics_->getLegsStandingPositions(), poseBody_);
    sendServoRequest(msg.duration_s);
}

void CRequester::requestTestLegs(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::TESTLEGS;

    for (const auto& [legIndex, leg] : kinematics_->getLegs()) {
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "CRequester:: requestTestLegs: " << magic_enum::enum_name(legIndex));

        CLegAngles legAngles = leg.angles_deg_;
        CLegAngles origLegAngles = legAngles;
        legAngles.femur_deg += 10.0;
        legAngles.tibia_deg += 10.0;
        legAngles.coxa_deg += 10.0;
        kinematics_->setLegAngles(legIndex, legAngles);
        sendServoRequest(msg.duration_s / 3.0);

        sendServoRequest(msg.duration_s / 3.0);

        kinematics_->setLegAngles(legIndex, origLegAngles);
        sendServoRequest(msg.duration_s / 3.0);
    }
}

// ------------------------------------------------------------------------------------------------------------
// public methods for the CRequester class
// ------------------------------------------------------------------------------------------------------------
void CRequester::onMovementTypeRequest(const MovementRequest& msg) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CRequester::onMovementRequest: " << msg.name);

    // Sequence is requested
    if (msg.duration_s > 0.0) {
        // setCallbackDuration(msg.duration_s);
        // set the active request type to NO_REQUEST in the callback
    }

    // check that the new request is inside the requestHandlers_ map
    if (requestHandlers_.find(msg.type) == requestHandlers_.end()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "CRequester::onMovementRequest: " << msg.name << " not known");
        return;
    }
    requestHandlers_[msg.type](msg);
}

void CRequester::onMovementVelocityRequest(const geometry_msgs::msg::Twist& msg) {
    velocity_ = msg;
}

void CRequester::onMovementBodyPoseRequest(const nikita_interfaces::msg::Pose& msg) {
    poseBody_ = msg;
}

void CRequester::update(std::chrono::milliseconds timeslice) {
    if (gaitController_->updateSelectedGait(velocity_, poseBody_)) {
        double duration = double(timeslice.count() / 1000.0);
        sendServoRequest(duration, false);
    }
}