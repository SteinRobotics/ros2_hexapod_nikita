#include "requester/gait_look.hpp"

#include <cmath>

namespace nikita_movement {

constexpr double DEFAULT_PHASE_INCREMENT = 0.05;
constexpr double EPSILON_PHASE = 1e-9;

CGaitHeadLook::CGaitHeadLook(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(node), kinematics_(kinematics) {
    kBodyMaxYaw_ = node_->declare_parameter<double>("GAIT_LOOK_BODY_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
    kHeadMaxYaw_ = node_->declare_parameter<double>("GAIT_LOOK_HEAD_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
}

void CGaitHeadLook::start(double /*duration_s*/, uint8_t direction) {
    state_ = EGaitState::Starting;
    phase_ = 0.0;
    amplitude_deg_ = (direction == 0) ? kHeadMaxYaw_ : -kHeadMaxYaw_;
    speed_ = 1.0;
}

bool CGaitHeadLook::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) return false;

    // advance phase according to speed
    double delta = DEFAULT_PHASE_INCREMENT * speed_;
    phase_ += delta;

    if (state_ == EGaitState::Starting) {
        // once we've got a non-trivial phase, become Running
        if (phase_ > EPSILON_PHASE) state_ = EGaitState::Running;
    }

    // set head yaw using sinusoidal oscillation
    kinematics_->getHead().yaw_deg = amplitude_deg_ * std::sin(phase_);

    // If stop pending -> transition to stopping then to stopped when phase wraps near zero
    if (state_ == EGaitState::StopPending) {
        state_ = EGaitState::Stopping;
    }
    if (state_ == EGaitState::Stopping) {
        // bring phase back toward zero for a clean stop (simple approach: reduce phase increment)
        phase_ *= 0.9;
        if (std::abs(phase_) < 1e-3) {
            state_ = EGaitState::Stopped;
            return false;
        }
    }

    return true;
}

void CGaitHeadLook::requestStop() {
    if (state_ == EGaitState::Running || state_ == EGaitState::Starting) {
        state_ = EGaitState::StopPending;
    }
}

void CGaitHeadLook::cancelStop() {
    if (state_ == EGaitState::StopPending || state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
