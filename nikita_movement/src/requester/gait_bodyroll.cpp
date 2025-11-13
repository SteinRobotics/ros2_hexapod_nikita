#include "requester/gait_bodyroll.hpp"

#include "requester/kinematics.hpp"

namespace nikita_movement {

CBodyRollGait::CBodyRollGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(std::move(node)), kinematics_(std::move(kinematics)) {
}

void CBodyRollGait::start() {
    state_ = EGaitState::Starting;
    phase_ = 0.0;
    state_ = EGaitState::Running;
}

void CBodyRollGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Stopped;
        return;
    }
    if (state_ == EGaitState::Stopped) {
        return;
    }
    constexpr double deltaPhase = 0.1;
    phase_ += deltaPhase;
    phase_ = std::fmod(phase_, TWO_PI);

    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (state_ == EGaitState::Starting && phase_ > M_PI_4) {
        state_ = EGaitState::Running;
    }
    const auto baseFootPos = kinematics_->getLegsStandingPositions();

    auto body = CPose();

    body.orientation.roll = kinematics_->BODY_MAX_ROLL * std::sin(phase_);

    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (state_ == EGaitState::Starting) {
        body.orientation.pitch = kinematics_->BODY_MAX_PITCH * std::sin(phase_);
    } else {
        body.orientation.pitch = kinematics_->BODY_MAX_PITCH * std::cos(phase_);
    }

    kinematics_->moveBody(baseFootPos, body);
}

void CBodyRollGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CBodyRollGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
