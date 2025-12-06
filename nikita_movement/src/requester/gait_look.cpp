#include "requester/gait_look.hpp"

#include <cmath>

namespace nikita_movement {

constexpr double DEFAULT_PHASE_INCREMENT = 0.05;
constexpr double EPSILON_PHASE = 1e-9;

CGaitLook::CGaitLook(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(node), kinematics_(kinematics) {
    RCLCPP_INFO(node_->get_logger(), "Initializing CGaitLook");
    kBodyMaxYaw_ = node_->declare_parameter<double>("GAIT_LOOK_BODY_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
    kHeadMaxYaw_ = node_->declare_parameter<double>("GAIT_LOOK_HEAD_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
}

void CGaitLook::start(double /*duration_s*/, uint8_t direction) {
    RCLCPP_INFO(node_->get_logger(), "Starting CGaitLook");
    state_ = EGaitState::Running;
    phase_ = 0.0;
    amplitude_deg_ = (direction == 0) ? kHeadMaxYaw_ : -kHeadMaxYaw_;
    speed_ = 1.0;
}

bool CGaitLook::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) return false;

    double delta = DEFAULT_PHASE_INCREMENT * speed_;
    phase_ += delta;
    // set head yaw using sinusoidal oscillation
    CHead head_request;
    head_request.yaw_deg = amplitude_deg_ * std::sin(phase_);
    kinematics_->setHead(head_request);

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

void CGaitLook::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::StopPending;
    }
}

void CGaitLook::cancelStop() {
    if (state_ == EGaitState::StopPending || state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
