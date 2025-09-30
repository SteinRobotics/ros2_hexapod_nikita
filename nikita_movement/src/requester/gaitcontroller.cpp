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

void CGaitController::setPhaseNeutral() {
    phase_ = 0.0;
}

void CGaitController::liftLegsTripodGroup(bool isFirstTripod) {
    auto positionLiftedLeg = kinematics_->getLegsPositions();

    const auto groupLiftedLegs = isFirstTripod ? groupFirstTripod_ : groupSecondTripod_;

    for (auto& [legIndex, position] : positionLiftedLeg) {
        if (std::find(groupLiftedLegs.begin(), groupLiftedLegs.end(), legIndex) != groupLiftedLegs.end()) {
            const auto baseFootPos = kinematics_->getLegsStandingPositions().at(legIndex);
            auto liftHight = baseFootPos.z + LEG_LIFT_HEIGHT;
            positionLiftedLeg[legIndex] = CPosition(position.x, position.y, liftHight);
        }
    }
    kinematics_->moveBody(positionLiftedLeg);
}

void CGaitController::updateCombinedTripodGait(const geometry_msgs::msg::Twist& velocity, CPose body) {
    // low-pass filtering the velocity
    const double alpha = 0.2;  // Adjust alpha for filtering strength (0.0 to 1.0)
    velocity_ = lowPassFilterTwist(velocity_, velocity, alpha);
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
    phase_ += deltaPhase;

    // RCLCPP_INFO(node_->get_logger(), "CGaitController::updateTripodGait: phase: %.4f", phase_);

    std::map<ELegIndex, CPosition> targetPositions;

    for (auto& [index, leg] : kinematics_->getLegs()) {
        bool isFirstTripod =
            std::find(groupFirstTripod_.begin(), groupFirstTripod_.end(), index) != groupFirstTripod_.end();

        double phaseOffset = isFirstTripod ? 0.0 : M_PI;
        double phaseWithOffset = phase_ + phaseOffset + M_PI_2;

        double step = GAIT_STEP_LENGTH * cos(phaseWithOffset);
        double lift = LEG_LIFT_HEIGHT * std::max(0.0, sin(phaseWithOffset));

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
