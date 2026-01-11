#include "requester/gait_ripple.hpp"

#include "requester/kinematics.hpp"

namespace nikita_movement {

CRippleGait::CRippleGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                         Parameters::Ripple& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CRippleGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    state_ = EGaitState::Starting;
    state_ = EGaitState::Running;
}

bool CRippleGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/,
                         const COrientation& /*head*/) {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Stopped;
        return false;
    }
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    // TODO: implement ripple gait
    return true;
}

void CRippleGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CRippleGait::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
