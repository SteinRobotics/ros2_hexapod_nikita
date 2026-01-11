#include "requester/gait_bodypose.hpp"

using namespace nikita_interfaces::msg;
namespace nikita_movement {

CGaitBodyPose::CGaitBodyPose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                             Parameters::BodyPose& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CGaitBodyPose::start(double duration_s, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CGaitBodyPose");
    state_ = EGaitState::Running;
    phase_ = 0.0;
    duration_s_ = duration_s;

    body_origin_ = kinematics_->getBody();
    origin_head_ = kinematics_->getHead();

    // 100ms task update time, duration in seconds,
    phase_increment_ = duration_s_ * 0.1;
}

bool CGaitBodyPose::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& body,
                           const COrientation& head) {
    if (state_ == EGaitState::Stopped) return false;

    // set head yaw using sinusoidal oscillation
    phase_ += phase_increment_;
    if (phase_ > duration_s_) {
        state_ = EGaitState::Stopped;
        kinematics_->setHead(CHead(head));
        kinematics_->moveBody(body);
        return true;
    }
    // interpolate body position from origin to target
    auto progress = phase_ / duration_s_;
    CPose intermediate_pose = body_origin_.linearInterpolate(body, progress);
    CHead target_head = CHead(head);
    CHead intermediate_head = origin_head_.linearInterpolate(target_head, progress);

    kinematics_->moveBody(intermediate_pose);
    kinematics_->setHead(intermediate_head);

    return true;
}

void CGaitBodyPose::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CGaitBodyPose::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace nikita_movement
