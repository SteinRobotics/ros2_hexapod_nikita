/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "handler/servohandler.hpp"

#include <rclcpp/create_timer.hpp>

using namespace std::chrono_literals;
using namespace nikita_interfaces::msg;

CServoHandler::CServoHandler(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    msgServoRequest_.target_angles.at(ServoIndex::HEAD_YAW).name = "HEAD_YAW";
    msgServoRequest_.target_angles.at(ServoIndex::HEAD_PITCH).name = "HEAD_PITCH";

    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_FRONT_COXA).name = "LEG_LEFT_FRONT_COXA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_FRONT_FEMUR).name = "LEG_LEFT_FRONT_FEMUR";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_FRONT_TIBIA).name = "LEG_LEFT_FRONT_TIBIA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_MID_COXA).name = "LEG_LEFT_MID_COXA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_MID_FEMUR).name = "LEG_LEFT_MID_FEMUR";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_MID_TIBIA).name = "LEG_LEFT_MID_TIBIA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_BACK_COXA).name = "LEG_LEFT_BACK_COXA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_BACK_FEMUR).name = "LEG_LEFT_BACK_FEMUR";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_BACK_TIBIA).name = "LEG_LEFT_BACK_TIBIA";

    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_FRONT_COXA).name = "LEG_RIGHT_FRONT_COXA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_FRONT_FEMUR).name = "LEG_RIGHT_FRONT_FEMUR";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_FRONT_TIBIA).name = "LEG_RIGHT_FRONT_TIBIA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_MID_COXA).name = "LEG_RIGHT_MID_COXA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_MID_FEMUR).name = "LEG_RIGHT_MID_FEMUR";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_MID_TIBIA).name = "LEG_RIGHT_MID_TIBIA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_BACK_COXA).name = "LEG_RIGHT_BACK_COXA";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_BACK_FEMUR).name = "LEG_RIGHT_BACK_FEMUR";
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_BACK_TIBIA).name = "LEG_RIGHT_BACK_TIBIA";

    servoController_ = std::make_shared<CServoController>(node_);

    callbackTimer_ = std::make_unique<CCallbackTimer>();

    pubServoRequest_ = node_->create_publisher<nikita_interfaces::msg::ServoRequest>("servo_request", 10);
}

void CServoHandler::run(CRequest request, bool blocking) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CServoHandler::run | CRequest");
    // set head angles
    msgServoRequest_.target_angles.at(ServoIndex::HEAD_YAW).angle_deg = request.head().degYaw;
    msgServoRequest_.target_angles.at(ServoIndex::HEAD_PITCH).angle_deg = request.head().degPitch;

    // set leg angles
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_FRONT_COXA).angle_deg =
        request.legAngles().at(ELegIndex::LeftFront).degCoxa;
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_FRONT_FEMUR).angle_deg =
        request.legAngles().at(ELegIndex::LeftFront).degFemur;
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_FRONT_TIBIA).angle_deg =
        request.legAngles().at(ELegIndex::LeftFront).degTibia;

    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_MID_COXA).angle_deg =
        request.legAngles().at(ELegIndex::LeftMid).degCoxa;
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_MID_FEMUR).angle_deg =
        request.legAngles().at(ELegIndex::LeftMid).degFemur;
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_MID_TIBIA).angle_deg =
        request.legAngles().at(ELegIndex::LeftMid).degTibia;

    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_BACK_COXA).angle_deg =
        request.legAngles().at(ELegIndex::LeftBack).degCoxa;
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_BACK_FEMUR).angle_deg =
        request.legAngles().at(ELegIndex::LeftBack).degFemur;
    msgServoRequest_.target_angles.at(ServoIndex::LEG_LEFT_BACK_TIBIA).angle_deg =
        request.legAngles().at(ELegIndex::LeftBack).degTibia;

    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_FRONT_COXA).angle_deg =
        request.legAngles().at(ELegIndex::RightFront).degCoxa;
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_FRONT_FEMUR).angle_deg =
        request.legAngles().at(ELegIndex::RightFront).degFemur;
    msgServoRequest_.target_angles.at(ServoIndex::LEG_RIGHT_FRONT_TIBIA).angle_deg =
        request.legAngles().at(ELegIndex::RightFront).degTibia;

    msgServoRequest_.header.stamp = node_->now();
    msgServoRequest_.time_to_reach_target_angles_ms = request.duration() * 1000.0;
    pubServoRequest_->publish(msgServoRequest_);

    servoController_->sendServoRequest(msgServoRequest_);

    if (blocking) {
        callbackTimer_->start(uint32_t(request.duration() * 1000),
                              std::bind(&CServoHandler::timerCallback, this), true);
    }
}

void CServoHandler::timerCallback() {
    executeNextPendingRequest();
}

// new methods
// ------------------------------------------------------------------------------------------------------------

void CServoHandler::requestWithoutQueue(CRequest request) {
    if (!pendingRequests_.empty()) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "CServoHandler:: requestWithoutQueue but queue is not empty");
        cancelRunningRequest();
    }
    run(request, false);
}

void CServoHandler::appendRequest(CRequest request) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CServoHandler:: new request ");
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
