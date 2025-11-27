#include "requester/gait_laydown.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "requester/kinematics.hpp"

constexpr double kPhaseIncrement = 0.10;
constexpr double kPhaseLimit = M_PI_2;

namespace nikita_movement {

CLayDownGait::CLayDownGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(std::move(node)), kinematics_(std::move(kinematics)) {
}

void CLayDownGait::start() {
    RCLCPP_INFO(node_->get_logger(), "CLayDownGait::start called, beginning laydown gait.");
    phase_ = 0.0;
    origin_positions_ = kinematics_->getLegsPositions();
    state_ = EGaitState::Running;
}

bool CLayDownGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& body) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    phase_ += kPhaseIncrement;
    if (phase_ > kPhaseLimit) {
        phase_ = kPhaseLimit;
        RCLCPP_INFO(node_->get_logger(), "CLayDownGait::update completed, laydown finished.");
        state_ = EGaitState::Stopped;
    }

    // progress in [0,1]
    const double progress = std::sin(phase_);
    const auto laydown_targets = kinematics_->getLegsLayDownPositions();

    // progress 0 means origin position
    // progress 1 means laydown position
    std::map<ELegIndex, CPosition> target_positions;
    for (const auto& [legIndex, laydown_pos] : laydown_targets) {
        auto origin_pos = origin_positions_.at(legIndex);
        CPosition blendedPos;
        blendedPos.x = nikita_utils::linearInterpolate(origin_pos.x, laydown_pos.x, progress);
        blendedPos.y = nikita_utils::linearInterpolate(origin_pos.y, laydown_pos.y, progress);
        blendedPos.z = nikita_utils::linearInterpolate(origin_pos.z, laydown_pos.z, progress);
        target_positions[legIndex] = blendedPos;
    }
    kinematics_->moveBody(target_positions, body);
    return true;
}

void CLayDownGait::requestStop() {
    RCLCPP_WARN(node_->get_logger(),
                "CLayDownGait::requestStop called, but Gait LayDown cannot be stopped once started.");
}

void CLayDownGait::cancelStop() {
}

}  // namespace nikita_movement
