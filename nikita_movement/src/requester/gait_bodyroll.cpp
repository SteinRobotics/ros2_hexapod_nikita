#include "requester/gait_bodyroll.hpp"

#include "requester/kinematics.hpp"

namespace nikita_movement {

CBodyRollGait::CBodyRollGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(std::move(node)), kinematics_(std::move(kinematics)) {
}

void CBodyRollGait::start() {
    state_ = EGaitState::Starting;
    phase_ = 0.0;
}

bool CBodyRollGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    constexpr double kDeltaPhase = 0.05;
    phase_ += kDeltaPhase;
    phase_ = std::fmod(phase_, TWO_PI);

    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (state_ == EGaitState::Starting && phase_ > M_PI_4) {
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::StopPending && utils::areSinCosValuesEqual(phase_, kDeltaPhase)) {
        state_ = EGaitState::Stopping;
    }

    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, kDeltaPhase)) {
        phase_ = 0.0;
        state_ = EGaitState::Stopped;
    }

    const auto base_foot_pos = kinematics_->getLegsStandingPositions();

    auto body = CPose();

    // Roll is always a sine wave
    body.orientation.roll_deg = kinematics_->BODY_MAX_ROLL * std::sin(phase_);

    // Pitch behavior depends on state
    if (state_ == EGaitState::Running || state_ == EGaitState::StopPending) {
        body.orientation.pitch_deg = kinematics_->BODY_MAX_PITCH * std::cos(phase_);
    } else {
        // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
        body.orientation.pitch_deg = kinematics_->BODY_MAX_PITCH * std::sin(phase_);
    }

    kinematics_->moveBody(base_foot_pos, body);
    return true;
}

void CBodyRollGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::StopPending;
    }
}

void CBodyRollGait::cancelStop() {
    // if the state is not in state StopPending, cancel the transition to Stop is not possible
    if (state_ == EGaitState::StopPending) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
