#include "requester/gait_waiting.hpp"

#include "requester/kinematics.hpp"

namespace nikita_movement {

void CWaitingGait::start() {
    // Directly enter Running; no distinct Starting phase needed.
    state_ = EGaitState::Running;
    phase_ = 0.0;
}

bool CWaitingGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    constexpr double deltaPhase = 0.1;
    phase_ += deltaPhase;

    // When stopping, finish current lift cycle cleanly.
    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, deltaPhase)) {
        state_ = EGaitState::Stopped;
        return false;
    }

    const auto baseFootPos = kinematics_->getLegsStandingPositions();
    auto bodyTarget = CPose();

    const auto kBodyLiftHeight = 0.05;                           // 5 cm body lift for visual effect
    bodyTarget.position.z = kBodyLiftHeight * std::sin(phase_);  // Small body bounce for visual effect

    kinematics_->moveBody(baseFootPos, bodyTarget);
    return true;
}

void CWaitingGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CWaitingGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
