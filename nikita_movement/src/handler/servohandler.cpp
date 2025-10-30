/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "handler/servohandler.hpp"

#include <rclcpp/create_timer.hpp>

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

CServoHandler::CServoHandler(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    servoController_ = std::make_shared<CServoController>(node_);
    simpleTimer_ = std::make_unique<CSimpleTimer>();
}

void CServoHandler::run(CRequest request, bool blocking) {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CServoHandler::run | CRequest");
    auto targetAngles = std::map<uint32_t, double>();

    // set head angles
    targetAngles[ServoIndex::HEAD_YAW] = request.head().degYaw;
    targetAngles[ServoIndex::HEAD_PITCH] = request.head().degPitch;

    // set leg angles
    targetAngles[ServoIndex::LEG_RIGHT_FRONT_COXA] = request.legAngles().at(ELegIndex::RightFront).degCoxa;
    targetAngles[ServoIndex::LEG_RIGHT_FRONT_FEMUR] = request.legAngles().at(ELegIndex::RightFront).degFemur;
    targetAngles[ServoIndex::LEG_RIGHT_FRONT_TIBIA] = request.legAngles().at(ELegIndex::RightFront).degTibia;

    targetAngles[ServoIndex::LEG_RIGHT_MID_COXA] = request.legAngles().at(ELegIndex::RightMid).degCoxa;
    targetAngles[ServoIndex::LEG_RIGHT_MID_FEMUR] = request.legAngles().at(ELegIndex::RightMid).degFemur;
    targetAngles[ServoIndex::LEG_RIGHT_MID_TIBIA] = request.legAngles().at(ELegIndex::RightMid).degTibia;

    targetAngles[ServoIndex::LEG_RIGHT_BACK_COXA] = request.legAngles().at(ELegIndex::RightBack).degCoxa;
    targetAngles[ServoIndex::LEG_RIGHT_BACK_FEMUR] = request.legAngles().at(ELegIndex::RightBack).degFemur;
    targetAngles[ServoIndex::LEG_RIGHT_BACK_TIBIA] = request.legAngles().at(ELegIndex::RightBack).degTibia;

    targetAngles[ServoIndex::LEG_LEFT_FRONT_COXA] = request.legAngles().at(ELegIndex::LeftFront).degCoxa;
    targetAngles[ServoIndex::LEG_LEFT_FRONT_FEMUR] = request.legAngles().at(ELegIndex::LeftFront).degFemur;
    targetAngles[ServoIndex::LEG_LEFT_FRONT_TIBIA] = request.legAngles().at(ELegIndex::LeftFront).degTibia;

    targetAngles[ServoIndex::LEG_LEFT_MID_COXA] = request.legAngles().at(ELegIndex::LeftMid).degCoxa;
    targetAngles[ServoIndex::LEG_LEFT_MID_FEMUR] = request.legAngles().at(ELegIndex::LeftMid).degFemur;
    targetAngles[ServoIndex::LEG_LEFT_MID_TIBIA] = request.legAngles().at(ELegIndex::LeftMid).degTibia;

    targetAngles[ServoIndex::LEG_LEFT_BACK_COXA] = request.legAngles().at(ELegIndex::LeftBack).degCoxa;
    targetAngles[ServoIndex::LEG_LEFT_BACK_FEMUR] = request.legAngles().at(ELegIndex::LeftBack).degFemur;
    targetAngles[ServoIndex::LEG_LEFT_BACK_TIBIA] = request.legAngles().at(ELegIndex::LeftBack).degTibia;

    servoController_->requestAngles(targetAngles, request.duration());

    if (blocking) {
        simpleTimer_->waitSecondsNonBlocking(request.duration(),
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
    if (!simpleTimer_->isRunning()) executeNextPendingRequest();
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
    simpleTimer_->stop();
}
