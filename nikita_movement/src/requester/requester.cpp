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

    if (auto controller = servoHandler_->getServoController()) {
        controller->setInitialAnglesCallback([this](const std::map<ELegIndex, CLegAngles>& initial_angles) {
            for (const auto& [leg_index, leg_angles] : initial_angles) {
                kinematics_->setLegAngles(leg_index, leg_angles);
            }
        });
    }

    initialize_request_handlers();

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
void CRequester::initialize_request_handlers() {
    auto bind = [this](auto fn) { return [this, fn](const MovementRequest& msg) { (this->*fn)(msg); }; };

    request_handlers_ = {
        {MovementRequest::NO_REQUEST,
         [this](const MovementRequest&) { activeRequest_ = MovementRequest::NO_REQUEST; }},
        {MovementRequest::LAYDOWN, bind(&CRequester::requestGait)},
        {MovementRequest::STAND_UP, bind(&CRequester::requestGait)},
        {MovementRequest::WAITING, bind(&CRequester::requestGait)},
        {MovementRequest::MOVE_TRIPOD, bind(&CRequester::requestGait)},
        {MovementRequest::WATCH, bind(&CRequester::requestGait)},
        {MovementRequest::LOOK_LEFT, bind(&CRequester::requestGait)},
        {MovementRequest::LOOK_RIGHT, bind(&CRequester::requestGait)},
        {MovementRequest::DANCE, bind(&CRequester::requestDance)},
        {MovementRequest::HIGH_FIVE, bind(&CRequester::requestGait)},
        {MovementRequest::LEGS_WAVE, bind(&CRequester::requestGait)},
        {MovementRequest::BODY_ROLL, bind(&CRequester::requestGait)},
        {MovementRequest::BITE, bind(&CRequester::requestBite)},
        {MovementRequest::STOMP, bind(&CRequester::requestStomp)},
        {MovementRequest::CLAP, bind(&CRequester::requestGait)},
        {MovementRequest::TRANSPORT, bind(&CRequester::requestTransport)},
        {MovementRequest::TESTBODY, bind(&CRequester::requestTestBody)},
        {MovementRequest::TESTLEGS, bind(&CRequester::requestTestLegs)},
        {MovementRequest::NEUTRAL, bind(&CRequester::requestNeutral)},
        {MovementRequest::CALIBRATE, bind(&CRequester::requestCalibrate)},
        // Add more handlers as needed
    };
}

void CRequester::sendServoRequest(const double duration_s) {
    auto head = kinematics_->getHead();
    auto legs = kinematics_->getLegsAngles();
    servoHandler_->run(CRequest(head, legs, duration_s));
}

void CRequester::requestGait(const MovementRequest& msg) {
    activeRequest_ = msg.type;
    gaitController_->setGait(msg);
}

void CRequester::requestClap(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::CLAP;
    [[maybe_unused]] auto tmp = msg;  // Suppress unused variable warning
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

    for (auto& [leg_index, leg] : kinematics_->getLegs()) {
        kinematics_->setLegAngles(leg_index, CLegAngles(0.0, 30.0, -20.0));
    }
    sendServoRequest(msg.duration_s);
}

void CRequester::requestNeutral(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::NEUTRAL;
    // set the head to the neutral position
    kinematics_->setHead(0.0, 0.0);
    // set all legs to the neutral position
    for (auto& [leg_index, leg] : kinematics_->getLegs()) {
        kinematics_->setLegAngles(leg_index, CLegAngles(0.0, 0.0, 0.0));
    }
    sendServoRequest(msg.duration_s);
}

void CRequester::requestCalibrate(const MovementRequest& msg) {
    activeRequest_ = MovementRequest::CALIBRATE;

    // set all legs to the kiss ground position
    auto positions_kiss_ground = kinematics_->getLegsStandingPositions();
    for (auto& [leg_index, position] : positions_kiss_ground) {
        positions_kiss_ground[leg_index] = CPosition(position.x, position.y, 0.0);
    }
    kinematics_->moveBody(positions_kiss_ground);
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

    for (const auto& [leg_index, leg] : kinematics_->getLegs()) {
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "CRequester:: requestTestLegs: " << magic_enum::enum_name(leg_index));

        CLegAngles leg_angles = leg.angles_deg_;
        CLegAngles orig_leg_angles = leg_angles;
        leg_angles.femur_deg += 10.0;
        leg_angles.tibia_deg += 10.0;
        leg_angles.coxa_deg += 10.0;
        kinematics_->setLegAngles(leg_index, leg_angles);
        sendServoRequest(msg.duration_s / 3.0);

        sendServoRequest(msg.duration_s / 3.0);

        kinematics_->setLegAngles(leg_index, orig_leg_angles);
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

    // check that the new request is inside the request_handlers_ map
    if (request_handlers_.find(msg.type) == request_handlers_.end()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "CRequester::onMovementRequest: " << msg.name << " not known");
        return;
    }
    request_handlers_[msg.type](msg);
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
        sendServoRequest(duration);
    }
}