/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/gaitcontroller.hpp"

// Concrete gait implementations
#include "requester/gait_bodyroll.hpp"
#include "requester/gait_highfive.hpp"
#include "requester/gait_laydown.hpp"
#include "requester/gait_legwave.hpp"
#include "requester/gait_ripple.hpp"
#include "requester/gait_standup.hpp"
#include "requester/gait_tripod.hpp"
#include "requester/gait_waiting.hpp"
#include "requester/gait_watch.hpp"

namespace {

using MovementRequestMsg = nikita_interfaces::msg::MovementRequest;

std::string movementRequestToString(MovementRequestMsg::_type_type type) {
    switch (type) {
        case MovementRequestMsg::NO_REQUEST:
            return "NO_REQUEST";
        case MovementRequestMsg::MOVE_TRIPOD:
            return "MOVE_TRIPOD";
        case MovementRequestMsg::WATCH:
            return "WATCH";
        case MovementRequestMsg::WAITING:
            return "WAITING";
        case MovementRequestMsg::STAND_UP:
            return "STAND_UP";
        case MovementRequestMsg::LAYDOWN:
            return "LAYDOWN";
        case MovementRequestMsg::HIGH_FIVE:
            return "HIGH_FIVE";
        case MovementRequestMsg::LEGS_WAVE:
            return "LEGS_WAVE";
        case MovementRequestMsg::BODY_ROLL:
            return "BODY_ROLL";
        default:
            return "UNKNOWN(" + std::to_string(type) + ")";
    }
}

}  // namespace

using namespace nikita_movement;

CGaitController::CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(node), kinematics_(kinematics) {
    kFactorVelocityToGaitCycleTime =
        node->declare_parameter<double>("FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME", rclcpp::PARAMETER_DOUBLE);
    kGaitStepLength = node->declare_parameter<double>("GAIT_STEP_LENGTH", rclcpp::PARAMETER_DOUBLE);
    kLegLiftHeight = node->declare_parameter<double>("LEG_LIFT_HEIGHT", rclcpp::PARAMETER_DOUBLE);

    // Create all gait instances
    gait_tripod_ = std::make_unique<CTripodGait>(node_, kinematics_, kLegLiftHeight, kGaitStepLength,
                                                 kFactorVelocityToGaitCycleTime);
    gait_ripple_ = std::make_unique<CRippleGait>(node_, kinematics_);
    gait_bodyroll_ = std::make_unique<CBodyRollGait>(node_, kinematics_);
    gait_legwave_ = std::make_unique<CGaitLegWave>(node_, kinematics_, kLegLiftHeight);
    gait_waiting_ = std::make_unique<CWaitingGait>(node_, kinematics_, kLegLiftHeight);
    gait_watch_ = std::make_unique<CWatchGait>(node_, kinematics_);
    gait_standup_ = std::make_unique<CStandUpGait>(node_, kinematics_);
    gait_laydown_ = std::make_unique<CLayDownGait>(node_, kinematics_);
    gait_highfive_ = std::make_unique<CHighFiveGait>(node_, kinematics_);

    // Default active gait
    // TODO change to Waiting?
    active_gait_ = gait_tripod_.get();
}

CGaitController::~CGaitController() = default;

void CGaitController::setGait(CGaitController::MovementRequestType type) {
    RCLCPP_INFO(node_->get_logger(), "CGaitController::setGait to request %s",
                movementRequestToString(type).c_str());

    assert(active_gait_);

    // If there's an active gait and the same gait is requested again, handle
    // transient states (Stopped -> start, Stopping -> cancelStop) and return.
    if (type == active_type_) {
        if (active_gait_->state() == EGaitState::Stopped) {
            active_gait_->start();
        }
        if (active_gait_->state() == EGaitState::Stopping) {
            active_gait_->cancelStop();
        }
        return;
    }

    // If the active gait is not yet stopped, request stop and set pending type
    if (active_gait_->state() != EGaitState::Stopped) {
        pending_type_ = type;
        active_gait_->requestStop();
        return;
    }

    if (pending_type_ != MovementRequestMsg::NO_REQUEST) {
        RCLCPP_WARN(node_->get_logger(),
                    "CGaitController::setGait: overriding pending gait %s with new request %s",
                    movementRequestToString(pending_type_).c_str(), movementRequestToString(type).c_str());
        pending_type_ = MovementRequestMsg::NO_REQUEST;
    }

    switchGait(type);
}

void CGaitController::switchGait(CGaitController::MovementRequestType type) {
    RCLCPP_INFO(node_->get_logger(), "CGaitController: switching to request %s",
                movementRequestToString(type).c_str());

    pending_type_ = MovementRequestMsg::NO_REQUEST;
    active_type_ = type;
    switch (type) {
        case MovementRequestMsg::MOVE_TRIPOD:
            active_gait_ = gait_tripod_.get();
            break;
        case MovementRequestMsg::BODY_ROLL:
            active_gait_ = gait_bodyroll_.get();
            break;
        case MovementRequestMsg::LEGS_WAVE:
            active_gait_ = gait_legwave_.get();
            break;
        case MovementRequestMsg::WAITING:
            active_gait_ = gait_waiting_.get();
            break;
        case MovementRequestMsg::WATCH:
            active_gait_ = gait_watch_.get();
            break;
        case MovementRequestMsg::STAND_UP:
            active_gait_ = gait_standup_.get();
            break;
        case MovementRequestMsg::LAYDOWN:
            active_gait_ = gait_laydown_.get();
            break;
        case MovementRequestMsg::HIGH_FIVE:
            active_gait_ = gait_highfive_.get();
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(),
                         "CGaitController::setGait: requested gait not available for request %s",
                         movementRequestToString(type).c_str());
            return;
    }

    active_gait_->start();
}

bool CGaitController::updateSelectedGait(const geometry_msgs::msg::Twist& velocity, CPose body) {
    assert(active_gait_);
    if (pending_type_ != MovementRequestMsg::NO_REQUEST && active_gait_->state() == EGaitState::Stopped) {
        switchGait(pending_type_);
    }
    return active_gait_->update(velocity, body);
}

void CGaitController::requestStopSelectedGait() {
    assert(active_gait_);
    active_gait_->requestStop();
}
