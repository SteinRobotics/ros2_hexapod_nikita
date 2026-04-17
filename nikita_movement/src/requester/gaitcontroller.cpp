/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/gaitcontroller.hpp"

using nikita_interfaces::msg::MovementRequest;

namespace nikita_movement {

nikita_interfaces::msg::MovementRequest CGaitController::createMsg(std::string name,
                                                                   MovementRequestType type) {
    nikita_interfaces::msg::MovementRequest msg;
    msg.name = name;
    msg.type = type;
    return msg;
}

CGaitController::CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(node), kinematics_(kinematics) {
    // read parameters
    params_ = Parameters::declare(node_);

    // Create publisher for movement type
    movement_type_pub_ =
        node_->create_publisher<nikita_interfaces::msg::MovementRequest>("movement_type_actual", 10);

    // Create all gait instances
    gaits_[MovementRequest::BODY_ROLL] =
        std::make_shared<CGaitBodyRoll>(node_, kinematics_, params_.bodyRoll);
    gaits_[MovementRequest::CALIBRATE] =
        std::make_shared<CGaitSinglePose>(node_, kinematics_, params_.singlePose);
    gaits_[MovementRequest::CLAP] = std::make_shared<CClapGait>(node_, kinematics_, params_.clap);
    gaits_[MovementRequest::HIGH_FIVE] =
        std::make_shared<CHighFiveGait>(node_, kinematics_, params_.highFive);
    gaits_[MovementRequest::LAYDOWN] = std::make_shared<CLayDownGait>(node_, kinematics_, params_.layDown);
    gaits_[MovementRequest::LEGS_WAVE] = std::make_shared<CGaitLegWave>(node_, kinematics_, params_.legWave);
    gaits_[MovementRequest::LOOK] = std::make_shared<CGaitLook>(node_, kinematics_, params_.look);
    gaits_[MovementRequest::MOVE] = std::make_shared<CMoveCombinedGait>(
        node_, kinematics_, params_.wave, params_.ripple, params_.tripod, params_.moveCombined);
    gaits_[MovementRequest::NEUTRAL] =
        std::make_shared<CGaitSinglePose>(node_, kinematics_, params_.singlePose);
    gaits_[MovementRequest::STAND_UP] = std::make_shared<CStandUpGait>(node_, kinematics_, params_.standUp);
    gaits_[MovementRequest::TESTLEGS] = std::make_shared<CTestLegsGait>(node_, kinematics_, params_.testLegs);
    gaits_[MovementRequest::SINGLE_POSE] =
        std::make_shared<CGaitSinglePose>(node_, kinematics_, params_.singlePose);
    gaits_[MovementRequest::CONTINUOUS_POSE] =
        std::make_shared<CGaitContinuousPose>(node_, kinematics_, params_.continuousPose);
    gaits_[MovementRequest::WAITING] = std::make_shared<CWaitingGait>(node_, kinematics_, params_.waiting);
    gaits_[MovementRequest::WATCH] = std::make_shared<CGaitWatch>(node_, kinematics_, params_.watch);

    // Default active gait (robot starts laying down)
    active_gait_ = gaits_[MovementRequest::LAYDOWN];
    active_request_ = createMsg("LAYDOWN", MovementRequest::LAYDOWN);
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);
}

CGaitController::~CGaitController() = default;

void CGaitController::setGait(nikita_interfaces::msg::MovementRequest request) {
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);

    if (request.type == MovementRequest::NO_REQUEST) {
        RCLCPP_DEBUG(node_->get_logger(), "CGaitController::setGait ignoring NO_REQUEST message");
        return;
    }

    // If there's an active gait and the same gait is requested again, handle
    // transient states (Stopped -> start, Stopping -> cancelStop) and return.
    if (request.type == active_request_.type) {
        if (active_gait_->state() == EGaitState::Stopped) {
            RCLCPP_INFO(node_->get_logger(), "CGaitController::setGait starting %s", request.name.c_str());
            active_gait_->start(request.duration_s, request.direction);
        }
        if (active_gait_->state() == EGaitState::Stopping) {
            active_gait_->cancelStop();
        }
        return;
    }

    // If the active gait is not yet stopped, request stop and set pending type
    if (active_gait_->state() != EGaitState::Stopped) {
        pending_request_ = request;
        active_gait_->requestStop();
        return;
    }

    // Otherwise, switch immediately
    switchGait(request);
}

void CGaitController::switchGait(nikita_interfaces::msg::MovementRequest request) {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "CGaitController: switching to " << request.name << " (type=" << +request.type << ")");
    pending_request_ = createMsg("NO_REQUEST", MovementRequest::NO_REQUEST);
    active_request_ = request;

    active_gait_ = gaits_[request.type];
    active_gait_->start(request.duration_s, request.direction);

    // Publish the new movement type
    movement_type_pub_->publish(request);
}

bool CGaitController::updateSelectedGait(const geometry_msgs::msg::Twist& velocity, CPose body,
                                         COrientation head) {
    if (pending_request_.type != MovementRequest::NO_REQUEST &&
        active_gait_->state() == EGaitState::Stopped) {
        switchGait(pending_request_);
    }
    return active_gait_->update(velocity, body, head);
}

void CGaitController::requestStopSelectedGait() {
    active_gait_->requestStop();
}

}  // namespace nikita_movement
