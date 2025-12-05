#include "requester/gait_watch.hpp"

#include <algorithm>

#include "nikita_utils/linear_interpolation.hpp"
#include "requester/kinematics.hpp"

namespace {
constexpr double kPhaseIncrement = 0.05;
constexpr double kHeadYawDelta = 40.0;
constexpr double kBodyYawDelta = 20.0;
}  // namespace

namespace nikita_movement {

CWatchGait::CWatchGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(std::move(node)), kinematics_(std::move(kinematics)) {
}

void CWatchGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    initial_head_ = kinematics_->getHead();
    initial_body_pose_ = kinematics_->getBody();
    base_foot_positions_ = kinematics_->getLegsPositions();

    initial_target_ = composeTarget(0.0, 0.0, 0.0);
    left_target_ = composeTarget(-kHeadYawDelta, 0.0, -kBodyYawDelta);
    right_target_ = composeTarget(kHeadYawDelta, 0.0, kBodyYawDelta);

    state_ = EGaitState::Running;
    transitionToPhase(EPhase::ToLeft, initial_target_, left_target_);
}

bool CWatchGait::update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    phase_progress_ = std::min(phase_progress_ + kPhaseIncrement, 1.0);
    applyInterpolatedPose(start_target_, end_target_, phase_progress_);

    if (phase_progress_ >= 1.0 - 1e-6) {
        switch (phase_) {
            case EPhase::ToLeft:
                if (state_ == EGaitState::Stopping) {
                    transitionToPhase(EPhase::ReturnToOrigin, left_target_, initial_target_);
                } else {
                    transitionToPhase(EPhase::ToRight, left_target_, right_target_);
                }
                break;
            case EPhase::ToRight:
                transitionToPhase(EPhase::ReturnToOrigin, right_target_, initial_target_);
                break;
            case EPhase::ReturnToOrigin:
                state_ = EGaitState::Stopped;
                phase_ = EPhase::Finished;
                break;
            case EPhase::Idle:
            case EPhase::Finished:
                break;
        }
    }

    return state_ != EGaitState::Stopped;
}

void CWatchGait::requestStop() {
    if (state_ != EGaitState::Running) {
        return;
    }
    state_ = EGaitState::Stopping;

    if (phase_ == EPhase::ReturnToOrigin || phase_ == EPhase::Finished) {
        return;
    }

    const auto currentTarget = captureCurrentTarget();
    transitionToPhase(EPhase::ReturnToOrigin, currentTarget, initial_target_);
}

void CWatchGait::cancelStop() {
    if (state_ == EGaitState::Stopping && phase_ != EPhase::ReturnToOrigin) {
        state_ = EGaitState::Running;
    }
}

void CWatchGait::applyInterpolatedPose(const PoseTarget& from, const PoseTarget& to, double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);
    const double headYaw = nikita_utils::linearInterpolate(from.headYaw, to.headYaw, alpha);
    const double headPitch = nikita_utils::linearInterpolate(from.headPitch, to.headPitch, alpha);

    CPose bodyPose = initial_body_pose_;
    // interpolate orientation using member method on COrientation
    COrientation fromOri(from.bodyRoll, from.bodyPitch, from.bodyYaw);
    COrientation toOri(to.bodyRoll, to.bodyPitch, to.bodyYaw);
    bodyPose.orientation = fromOri.linearInterpolate(toOri, alpha);

    kinematics_->moveBody(base_foot_positions_, bodyPose);
    kinematics_->setHead(headYaw, headPitch);
}

void CWatchGait::transitionToPhase(EPhase next_phase, const PoseTarget& from, const PoseTarget& to) {
    phase_ = next_phase;
    phase_progress_ = 0.0;
    start_target_ = from;
    end_target_ = to;
}

CWatchGait::PoseTarget CWatchGait::composeTarget(double headYawDelta, double headPitchDelta,
                                                 double bodyYawDelta) const {
    PoseTarget target;
    target.headYaw = initial_head_.yaw_deg + headYawDelta;
    target.headPitch = initial_head_.pitch_deg + headPitchDelta;
    target.bodyRoll = initial_body_pose_.orientation.roll_deg;
    target.bodyPitch = initial_body_pose_.orientation.pitch_deg;
    target.bodyYaw = initial_body_pose_.orientation.yaw_deg + bodyYawDelta;
    return target;
}

CWatchGait::PoseTarget CWatchGait::captureCurrentTarget() const {
    const auto head = kinematics_->getHead();
    const auto body = kinematics_->getBody();
    return PoseTarget{head.yaw_deg, head.pitch_deg, body.orientation.roll_deg, body.orientation.pitch_deg,
                      body.orientation.yaw_deg};
}

}  // namespace nikita_movement
