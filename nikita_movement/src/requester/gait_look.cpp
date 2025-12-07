#include "requester/gait_look.hpp"

#include <cmath>

namespace nikita_movement {

constexpr double EPSILON_PHASE = 1e-9;

CGaitLook::CGaitLook(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(node), kinematics_(kinematics) {
    RCLCPP_INFO(node_->get_logger(), "Initializing CGaitLook");
    kHeadMaxYaw_ = node_->declare_parameter<double>("GAIT_LOOK_HEAD_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
    kBodyMaxYaw_ = node_->declare_parameter<double>("GAIT_LOOK_BODY_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
}

void CGaitLook::start(double duration_s, uint8_t direction) {
    RCLCPP_INFO(node_->get_logger(), "Starting CGaitLook");
    state_ = EGaitState::Running;
    phase_ = 0.0;
    amplitude_head_deg_ = (direction == MovementType::CLOCKWISE) ? kHeadMaxYaw_ : -kHeadMaxYaw_;
    amplitude_body_deg_ = (direction == MovementType::CLOCKWISE) ? kBodyMaxYaw_ : -kBodyMaxYaw_;

    // 100ms task update time, duration in seconds, 1 full cycle = 2pi
    delta_phase_ = (2.0 * M_PI) / (duration_s / 0.1);
}

bool CGaitLook::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) return false;

    // set head yaw using sinusoidal oscillation
    phase_ += delta_phase_;
    if (phase_ > 2.0 * M_PI) {
        state_ = EGaitState::Stopped;
        return false;
    }
    CHead head_request;
    head_request.yaw_deg = amplitude_head_deg_ * std::sin(phase_);
    kinematics_->setHead(head_request);

    CPose body_request;
    body_request.yaw_deg = amplitude_body_deg_ * std::sin(phase_);
    kinematics_->moveBody(body_request);

    return true;
}

void CGaitLook::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CGaitLook::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace nikita_movement
