#include "requester/gait_continuouspose.hpp"

using namespace nikita_interfaces::msg;
namespace nikita_movement {

CGaitContinuousPose::CGaitContinuousPose(std::shared_ptr<rclcpp::Node> node,
                                         std::shared_ptr<CKinematics> kinematics,
                                         Parameters::ContinuousPose& params)
    : node_(node), kinematics_(kinematics), params_(params) {
}

void CGaitContinuousPose::start(double /*duration_s*/, uint8_t /*direction*/) {
    RCLCPP_INFO(node_->get_logger(), "Starting CGaitContinuousPose");
    state_ = EGaitState::Running;

    body_origin_ = kinematics_->getBody();
    head_origin_ = kinematics_->getHead();
}

bool CGaitContinuousPose::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& body,
                                 const COrientation& head) {
    if (state_ == EGaitState::Stopped) return false;

    body_origin_ = body_origin_.linearInterpolate(body, 0.1);
    head_origin_ = head_origin_.linearInterpolate(head, 0.1);

    kinematics_->moveBody(body_origin_);
    kinematics_->setHead(head_origin_);
    return true;
}

void CGaitContinuousPose::requestStop() {
    // gait is stopped automatically after completing the current cycle
}

void CGaitContinuousPose::cancelStop() {
    // this gait cannot be stopped nor the stop can be cancelled
}

}  // namespace nikita_movement
