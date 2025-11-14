#include "requester/gait_legwave.hpp"

#include <magic_enum.hpp>

#include "requester/kinematics.hpp"

namespace nikita_movement {

void CGaitLegWave::start() {
    state_ = EGaitState::Running;
    phase_ = 0.0;
    activeLegIndex_ = ELegIndex::RightFront;
}

bool CGaitLegWave::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    constexpr double deltaPhase = 0.1;
    phase_ += deltaPhase;

    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, deltaPhase)) {
        state_ = EGaitState::Stopped;
        return false;
    }

    const auto baseFootPos = kinematics_->getLegsStandingPositions();

    if (phase_ >= M_PI) {
        // reset last leg to neutral position
        kinematics_->setSingleFeet(activeLegIndex_, baseFootPos.at(activeLegIndex_));

        // advance to the next leg
        const auto values = magic_enum::enum_values<ELegIndex>();
        const std::size_t idx = magic_enum::enum_index(activeLegIndex_).value();
        activeLegIndex_ = values[(idx + 1) % values.size()];
        phase_ = 0.0;
    }

    auto targetPosition = baseFootPos.at(activeLegIndex_);
    targetPosition.z = baseFootPos.at(activeLegIndex_).z + kLegLiftHeight * std::sin(phase_);

    kinematics_->setSingleFeet(activeLegIndex_, targetPosition);
    return true;
}

void CGaitLegWave::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CGaitLegWave::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
