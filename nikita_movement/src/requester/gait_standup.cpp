#include "requester/gait_standup.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "requester/kinematics.hpp"

constexpr double kPhaseIncrement = 0.10;
constexpr double kPhaseLimit = M_PI_2;

namespace nikita_movement {

CStandUpGait::CStandUpGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(std::move(node)), kinematics_(std::move(kinematics)) {
}

void CStandUpGait::start() {
    RCLCPP_INFO(node_->get_logger(), "CStandUpGait::start called, beginning standup gait.");
    phase_ = 0.0;
    origin_positions_ = kinematics_->getLegsPositions();
    state_ = EGaitState::Running;
}

bool CStandUpGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& body) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    phase_ += kPhaseIncrement;
    if (phase_ > kPhaseLimit) {
        phase_ = kPhaseLimit;
        RCLCPP_INFO(node_->get_logger(), "CStandUpGait::update completed, standup finished.");
        state_ = EGaitState::Stopped;
    }

    // progress in [0,1]
    const double progress = std::sin(phase_);
    const auto standingTargets = kinematics_->getLegsStandingPositions();

    // progress 0 means origin position
    // progress 1 means standing position
    std::map<ELegIndex, CPosition> target_positions;
    for (const auto& [legIndex, standing_position] : standingTargets) {
        auto origin_position = origin_positions_.at(legIndex);
        // use CPosition member interpolation instead of per-component scalar interpolation
        CPosition blendedPos = origin_position.linearInterpolate(standing_position, progress);
        target_positions[legIndex] = blendedPos;
    }
    kinematics_->moveBody(target_positions, body);
    return true;
}

void CStandUpGait::requestStop() {
    RCLCPP_WARN(node_->get_logger(),
                "CStandUpGait::requestStop called, but Gait StandUp cannot be stopped once started.");
}

void CStandUpGait::cancelStop() {
}

}  // namespace nikita_movement
