/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/gaitcontroller.hpp"

CGaitController::CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(node), kinematics_(kinematics) {
    FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME =
        node->declare_parameter<double>("FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME", rclcpp::PARAMETER_DOUBLE);
    GAIT_STEP_LENGTH = node->declare_parameter<double>("GAIT_STEP_LENGTH", rclcpp::PARAMETER_DOUBLE);
    LEG_LIFT_HEIGHT = node->declare_parameter<double>("LEG_LIFT_HEIGHT", rclcpp::PARAMETER_DOUBLE);
}

void CGaitController::resetPhase() {
    phase_ = 0.0;
    isStartPhaseDone_ = false;
    isLeaveCycleRequested_ = false;
}

void CGaitController::liftLegsTripodGroup(bool isFirstTripod) {
    auto positionLiftedLeg = kinematics_->getLegsPositions();

    const auto groupLiftedLegs = isFirstTripod ? groupFirstTripod_ : groupSecondTripod_;

    for (auto& [legIndex, position] : positionLiftedLeg) {
        if (std::ranges::find(groupLiftedLegs, legIndex) != groupLiftedLegs.end()) {
            const auto baseFootPos = kinematics_->getLegsStandingPositions().at(legIndex);
            auto liftHight = baseFootPos.z + LEG_LIFT_HEIGHT;
            positionLiftedLeg[legIndex] = CPosition(position.x, position.y, liftHight);
        }
    }
    kinematics_->moveBody(positionLiftedLeg);
}

void CGaitController::updateTripodGait(const geometry_msgs::msg::Twist& velocity, CPose body) {
    if (abs(velocity.linear.x) < 0.001 && abs(velocity.linear.y) < 0.001 && abs(velocity.angular.z) < 0.001) {
        return;
    }

    // low-pass filtering the velocity
    const double alpha = 0.2;  // Adjust alpha for filtering strength (0.0 to 1.0)
    velocity_ = utils::lowPassFilterTwist(velocity_, velocity, alpha);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CGaitController::requestMove: filtered velocity: "
    //                                             << velocity_.linear.x << ", " << velocity_.linear.y << ", "
    //                                             << velocity_.angular.z);

    double linear_x = velocity_.linear.x;
    double linear_y = velocity_.linear.y;
    double angular_z = velocity_.angular.z;

    constexpr double ROTATION_WEIGHT = 0.7;

    double combined_mag =
        std::sqrt((linear_x * linear_x) + (linear_y * linear_y) + (ROTATION_WEIGHT * angular_z * angular_z));

    // Avoid division by zero
    if (combined_mag < 1e-6) return;

    double norm_x = linear_x / combined_mag;
    double norm_y = linear_y / combined_mag;
    double norm_rot = angular_z / combined_mag;

    // the velocity is taken into account to calculate the gait cycle time with combined_mag
    double deltaPhase = FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME * combined_mag;
    // TODO handle overflow of phase_
    phase_ += deltaPhase;

    // RCLCPP_INFO(node_->get_logger(), "CGaitController::updateTripodGait: phase: %.4f", phase_);

    std::map<ELegIndex, CPosition> targetPositions;

    for (auto& [index, leg] : kinematics_->getLegs()) {
        bool isFirstTripodActive = std::ranges::find(groupFirstTripod_, index) != groupFirstTripod_.end();

        double phaseOffset = isFirstTripodActive ? 0.0 : M_PI;
        double phaseWithOffset = phase_ + phaseOffset + M_PI_2;

        double step = GAIT_STEP_LENGTH * cos(phaseWithOffset);
        double lift = LEG_LIFT_HEIGHT * std::max(0.0, sin(phaseWithOffset));

        // this if condition is only relevant for the first movement of the FirstTripod legs and only for 0 < phase_ < M_PI_4
        // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
        if (isFirstTripodActive && phase_ < M_PI_4) {
            lift = LEG_LIFT_HEIGHT * std::max(0.0, sin(phase_));
        }

        const auto baseFootPos = kinematics_->getLegsStandingPositions().at(index);

        // linear
        double deltaX = norm_x * step;
        double deltaY = norm_y * step;

        // rotation
        double legVecX = baseFootPos.x;
        double legVecY = baseFootPos.y;
        double len = std::sqrt(legVecX * legVecX + legVecY * legVecY);

        double rotX = 0.0;
        double rotY = 0.0;

        if (len > 1e-6) {
            double dirX = -legVecY / len;
            double dirY = legVecX / len;

            rotX = dirX * step * norm_rot;
            rotY = dirY * step * norm_rot;
        }

        CPosition target;
        target.x = baseFootPos.x + deltaX + rotX;
        target.y = baseFootPos.y + deltaY + rotY;
        target.z = baseFootPos.z + lift;

        targetPositions[index] = target;
    }

    kinematics_->moveBody(targetPositions, body);

    constexpr double MAX_HEAD_YAW_AMPLITUDE_DEG = 15.0;
    kinematics_->getHead().degYaw = MAX_HEAD_YAW_AMPLITUDE_DEG * std::sin(phase_);
}

void CGaitController::updateBodyRoll() {
    constexpr double deltaPhase = 0.1;
    phase_ += deltaPhase;
    phase_ = std::fmod(phase_, TWO_PI);

    const auto baseFootPos = kinematics_->getLegsStandingPositions();

    auto body = CPose();
    
    body.orientation.roll = kinematics_->BODY_MAX_ROLL * std::sin(phase_);

    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (phase_ < M_PI_4 && !isStartPhaseDone_) {
        body.orientation.pitch = kinematics_->BODY_MAX_PITCH * std::sin(phase_);
    } else {
        isStartPhaseDone_ = true;
        body.orientation.pitch = kinematics_->BODY_MAX_PITCH * std::cos(phase_);
    }

    // if ( isLeaveCycleRequested_ && phase_ >= M_PI + M_PI_4 ) {
    //     // Start returning to neutral
    //     body.orientation.pitch = kinematics_->BODY_MAX_PITCH * std::sin(phase_);
    // }

    // if (phase_ >= TWO_PI - deltaPhase) {
    //     // End of cycle reached, reset phase and stop further updates
    // }

    kinematics_->moveBody(baseFootPos, body);

}

void CGaitController::initLegRoll() {
    phase_ = 0.0;
    activeLegIndex_ = ELegIndex::RightFront;
}

void CGaitController::updateLegRoll() {

    constexpr double deltaPhase = 0.1;
    phase_ += deltaPhase;

    const auto baseFootPos = kinematics_->getLegsStandingPositions();

    if (phase_ >= M_PI) {
        // reset last leg to neutral position
        kinematics_->setSingleFeet(activeLegIndex_, baseFootPos.at(activeLegIndex_));

        //activate next leg
        activeLegIndex_ = (static_cast<ELegIndex>((static_cast<int>(activeLegIndex_) + 1) % 6));
        phase_ = 0.0;
    }

    auto targetPosition = baseFootPos.at(activeLegIndex_);
    targetPosition.z = baseFootPos.at(activeLegIndex_).z + LEG_LIFT_HEIGHT * std::sin(phase_);  // validate +

    kinematics_->setSingleFeet(activeLegIndex_, targetPosition);

}
