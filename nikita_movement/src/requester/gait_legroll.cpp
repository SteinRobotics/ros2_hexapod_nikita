#include "requester/gait_legroll.hpp"

#include "requester/kinematics.hpp"

namespace nikita_movement {

CLegRollGait::CLegRollGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(std::move(node)), kinematics_(std::move(kinematics)) {
}

void CLegRollGait::start() {
    // state_ = EGaitState::Starting;
    state_ = EGaitState::Running;
    phase_ = 0.0;
    activeLegIndex_ = ELegIndex::RightFront;
}

void CLegRollGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Stopped;
        return;
    }
    if (state_ == EGaitState::Stopped) {
        return;
    }
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

void CLegRollGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CLegRollGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
