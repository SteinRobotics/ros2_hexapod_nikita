#include "requester/gait_neutral.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "requester/types.hpp"

namespace {
constexpr double kPhaseIncrement = 0.10;
constexpr double kPhaseLimit = M_PI_2;
}  // namespace

namespace nikita_movement {

CNeutralGait::CNeutralGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(std::move(node)), kinematics_(std::move(kinematics)) {
}

void CNeutralGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    phase_ = 0.0;
    origin_leg_angles_ = kinematics_->getLegsAngles();
    target_leg_angles_.clear();
    for (const auto& [leg_index, _] : origin_leg_angles_) {
        target_leg_angles_[leg_index] = CLegAngles(0.0, 0.0, 0.0);
    }
    origin_head_ = kinematics_->getHead();
    target_head_ = CHead(0.0, 0.0);
    state_ = EGaitState::Running;
}

bool CNeutralGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    if (state_ == EGaitState::Stopping) {
        phase_ = kPhaseLimit;
    } else {
        phase_ = std::min(phase_ + kPhaseIncrement, kPhaseLimit);
    }

    const double progress = std::sin(phase_) * std::sin(phase_);
    applyNeutralPose(progress);

    if (phase_ >= kPhaseLimit - 1e-6) {
        state_ = EGaitState::Stopped;
    }

    return true;
}

void CNeutralGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CNeutralGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

void CNeutralGait::applyNeutralPose(double progress) {
    const double clamped = std::clamp(progress, 0.0, 1.0);
    for (const auto& [leg_index, origin_angles] : origin_leg_angles_) {
        const auto target_iter = target_leg_angles_.find(leg_index);
        if (target_iter == target_leg_angles_.end()) {
            continue;
        }
        const auto interpolated = origin_angles.linearInterpolate(target_iter->second, clamped);
        kinematics_->setLegAngles(leg_index, interpolated);
    }

    const auto head = origin_head_.linearInterpolate(target_head_, clamped);
    kinematics_->setHead(head);
}

}  // namespace nikita_movement
