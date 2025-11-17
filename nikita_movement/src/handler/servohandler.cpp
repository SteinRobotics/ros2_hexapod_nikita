/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "handler/servohandler.hpp"

#include <rclcpp/create_timer.hpp>

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

CServoHandler::CServoHandler(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    servoController_ = std::make_shared<CServoController>(node_);
    callbackTimer_ = std::make_unique<CCallbackTimer>();
}

void CServoHandler::run(CRequest request, bool blocking) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CServoHandler::run | CRequest");
    auto targetAngles = std::map<uint32_t, double>();

    // set head angles
    targetAngles[ServoIndex::HEAD_YAW] = request.head().yaw_deg;
    targetAngles[ServoIndex::HEAD_PITCH] = request.head().pitch_deg;

    // set leg angles
    targetAngles[ServoIndex::LEG_RIGHT_FRONT_COXA] = request.legAngles().at(ELegIndex::RightFront).coxa_deg;
    targetAngles[ServoIndex::LEG_RIGHT_FRONT_FEMUR] = request.legAngles().at(ELegIndex::RightFront).femur_deg;
    targetAngles[ServoIndex::LEG_RIGHT_FRONT_TIBIA] = request.legAngles().at(ELegIndex::RightFront).tibia_deg;

    targetAngles[ServoIndex::LEG_RIGHT_MID_COXA] = request.legAngles().at(ELegIndex::RightMid).coxa_deg;
    targetAngles[ServoIndex::LEG_RIGHT_MID_FEMUR] = request.legAngles().at(ELegIndex::RightMid).femur_deg;
    targetAngles[ServoIndex::LEG_RIGHT_MID_TIBIA] = request.legAngles().at(ELegIndex::RightMid).tibia_deg;

    targetAngles[ServoIndex::LEG_RIGHT_BACK_COXA] = request.legAngles().at(ELegIndex::RightBack).coxa_deg;
    targetAngles[ServoIndex::LEG_RIGHT_BACK_FEMUR] = request.legAngles().at(ELegIndex::RightBack).femur_deg;
    targetAngles[ServoIndex::LEG_RIGHT_BACK_TIBIA] = request.legAngles().at(ELegIndex::RightBack).tibia_deg;

    targetAngles[ServoIndex::LEG_LEFT_FRONT_COXA] = request.legAngles().at(ELegIndex::LeftFront).coxa_deg;
    targetAngles[ServoIndex::LEG_LEFT_FRONT_FEMUR] = request.legAngles().at(ELegIndex::LeftFront).femur_deg;
    targetAngles[ServoIndex::LEG_LEFT_FRONT_TIBIA] = request.legAngles().at(ELegIndex::LeftFront).tibia_deg;

    targetAngles[ServoIndex::LEG_LEFT_MID_COXA] = request.legAngles().at(ELegIndex::LeftMid).coxa_deg;
    targetAngles[ServoIndex::LEG_LEFT_MID_FEMUR] = request.legAngles().at(ELegIndex::LeftMid).femur_deg;
    targetAngles[ServoIndex::LEG_LEFT_MID_TIBIA] = request.legAngles().at(ELegIndex::LeftMid).tibia_deg;

    targetAngles[ServoIndex::LEG_LEFT_BACK_COXA] = request.legAngles().at(ELegIndex::LeftBack).coxa_deg;
    targetAngles[ServoIndex::LEG_LEFT_BACK_FEMUR] = request.legAngles().at(ELegIndex::LeftBack).femur_deg;
    targetAngles[ServoIndex::LEG_LEFT_BACK_TIBIA] = request.legAngles().at(ELegIndex::LeftBack).tibia_deg;

    servoController_->requestAngles(targetAngles, request.duration());

    if (blocking) {
        callbackTimer_->waitSecondsNonBlocking(request.duration(),
                                               std::bind(&CServoHandler::timerCallback, this));
    }
}

void CServoHandler::timerCallback() {
    executeNextPendingRequest();
}

void CServoHandler::requestWithoutQueue(CRequest request) {
    if (!pendingRequests_.empty()) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "CServoHandler:: requestWithoutQueue but queue is not empty");
        cancelRunningRequest();
    }
    run(request, false);
}

void CServoHandler::appendRequest(CRequest request) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CServoHandler:: new request ");
    pendingRequests_.push_back(request);
    if (!callbackTimer_->isRunning()) executeNextPendingRequest();
}

bool CServoHandler::isDone() {
    return pendingRequests_.empty();
}

void CServoHandler::executeNextPendingRequest() {
    if (pendingRequests_.empty()) {
        return;
    }
    auto request = pendingRequests_.front();
    pendingRequests_.pop_front();
    run(request);
}

void CServoHandler::cancelRunningRequest() {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CServoHandler::cancelRunningRequest");
    pendingRequests_.clear();
    callbackTimer_->stop();
}
