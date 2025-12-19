#include "requester/gait_calibrate.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace {
constexpr double kPhaseIncrement = 0.10;
constexpr double kPhaseLimit = M_PI_2;
}  // namespace

namespace nikita_movement {

CCalibrateGait::CCalibrateGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(std::move(node)), kinematics_(std::move(kinematics)) {
}

void CCalibrateGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    phase_ = 0.0;
    origin_leg_positions_ = kinematics_->getLegsPositions();
    target_leg_positions_ = kinematics_->getLegsStandingPositions();
    for (auto& [leg_index, position] : target_leg_positions_) {
        (void)leg_index;
        position.z = 0.0;
    }
    state_ = EGaitState::Running;
}

bool CCalibrateGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    if (state_ == EGaitState::Stopping) {
        phase_ = kPhaseLimit;
    } else {
        phase_ = std::min(phase_ + kPhaseIncrement, kPhaseLimit);
    }

    const double progress = std::sin(phase_) * std::sin(phase_);
    applyCalibratePose(progress);

    if (phase_ >= kPhaseLimit - 1e-6) {
        state_ = EGaitState::Stopped;
    }

    return true;
}

void CCalibrateGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CCalibrateGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

void CCalibrateGait::applyCalibratePose(double progress) {
    std::map<ELegIndex, CPosition> interpolated_positions;
    const double clamped = std::clamp(progress, 0.0, 1.0);
    for (const auto& [leg_index, origin_position] : origin_leg_positions_) {
        const auto target_iter = target_leg_positions_.find(leg_index);
        if (target_iter == target_leg_positions_.end()) {
            continue;
        }
        interpolated_positions[leg_index] = origin_position.linearInterpolate(target_iter->second, clamped);
    }
    kinematics_->moveBody(interpolated_positions);
}

}  // namespace nikita_movement
