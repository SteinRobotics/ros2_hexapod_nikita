/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/gaitcontroller.hpp"

// Concrete gait implementations
#include "requester/gait_bodyroll.hpp"
#include "requester/gait_legwave.hpp"
#include "requester/gait_ripple.hpp"
#include "requester/gait_tripod.hpp"

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

    // Default active gait
    // TODO change to Waiting?
    active_gait_ = gait_tripod_.get();
}

CGaitController::~CGaitController() = default;

void CGaitController::setGait(EGaitType type) {
    RCLCPP_INFO(node_->get_logger(), "CGaitController::setGait: switching to gait type %s",
                magic_enum::enum_name(type).data());

    if (!active_gait_) return;

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

    // Switch to the requested gait implementation.
    active_type_ = type;
    switch (type) {
        case EGaitType::Tripod:
            active_gait_ = gait_tripod_.get();
            break;
        case EGaitType::Ripple:
            active_gait_ = gait_ripple_.get();
            break;
        case EGaitType::BodyRoll:
            active_gait_ = gait_bodyroll_.get();
            break;
        case EGaitType::LegWave:
            active_gait_ = gait_legwave_.get();
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "CGaitController::setGait: requested gait not available %s",
                         magic_enum::enum_name(type).data());
            break;
    }

    active_gait_->start();
}

bool CGaitController::updateSelectedGait(const geometry_msgs::msg::Twist& velocity, CPose body) {
    if (!active_gait_) return false;
    return active_gait_->update(velocity, body);
}

void CGaitController::requestStopSelectedGait() {
    if (!active_gait_) return;
    active_gait_->requestStop();
}
