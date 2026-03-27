#include "requester/gait_wave.hpp"

namespace nikita_movement {

constexpr double kWaveTimeToWaitBeforeStopSec = 3.0;

CWaveGait::CWaveGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                     Parameters::Wave& params)
    : node_(node), kinematics_(kinematics), params_(params) {
    no_velocity_timer_.stop();
    body_old_ = CPose();
    target_positions_ = kinematics_->getLegsStandingPositions();
}

void CWaveGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    state_ = EGaitState::Starting;
    phase_ = 0.0;
    velocity_ = geometry_msgs::msg::Twist();
}

bool CWaveGait::update(const geometry_msgs::msg::Twist& velocity, const CPose& body,
                       const COrientation& /*head*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    if (utils::isTwistZero(velocity) && state_ == EGaitState::Running && body != body_old_) {
        const auto base_foot_pos = kinematics_->getLegsStandingPositions();
        kinematics_->moveBody(base_foot_pos, body);
        body_old_ = body;
        return true;
    }

    if (utils::isTwistZero(velocity) && state_ == EGaitState::Running) {
        if (!no_velocity_timer_.isRunning()) {
            no_velocity_timer_.start();
        } else if (no_velocity_timer_.haveSecondsElapsed(kWaveTimeToWaitBeforeStopSec)) {
            requestStop();
            no_velocity_timer_.stop();
        }
        return false;
    }

    // Reset idle timer when velocity resumes
    no_velocity_timer_.stop();

    // Filter and store the last non-zero velocity
    if (!utils::isTwistZero(velocity)) {
        velocity_ = utils::limitChangeRateUpTwist(velocity_, velocity, params_.velocity_filter_alpha);
    }

    double linear_x = velocity_.linear.x;
    double linear_y = velocity_.linear.y;
    double angular_z = velocity_.angular.z;

    double combined_mag = std::sqrt((linear_x * linear_x) + (linear_y * linear_y) +
                                    (params_.rotation_weight * angular_z * angular_z));

    if (combined_mag < 1e-6) return false;

    double norm_x = linear_x / combined_mag;
    double norm_y = linear_y / combined_mag;
    double norm_rot = angular_z / combined_mag;

    double delta_phase = params_.factor_velocity_to_gait_cycle_time * combined_mag;
    phase_ += delta_phase;
    phase_ = std::fmod(phase_, 2.0 * M_PI);

    // Transition from Starting to Running after a short initial phase
    if (state_ == EGaitState::Starting && phase_ > M_PI / 6.0) {
        state_ = EGaitState::Running;
    }

    // StopPending: wait for phase to be near a cycle boundary where all legs are in support
    if (state_ == EGaitState::StopPending && utils::isSinValueNearZero(phase_, delta_phase)) {
        RCLCPP_INFO(node_->get_logger(), "CWaveGait::update: Transitioning to Stopped state, phase_: %.2f",
                    phase_);
        phase_ = 0.0;
        state_ = EGaitState::Stopped;
        kinematics_->moveBody(kinematics_->getLegsStandingPositions(), body);
        kinematics_->setHead(COrientation(0.0, 0.0, 0.0));
        return true;
    }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "CWaveGait::update: state_: " << magic_enum::enum_name(state_));
    // RCLCPP_INFO(node_->get_logger(), "CWaveGait::update: phase_: %.2f, delta_phase: %.2f", phase_,
    //             delta_phase);

    const auto standing_positions = kinematics_->getLegsStandingPositions();

    for (const auto& leg_info : leg_phases_) {
        const auto index = leg_info.index;
        const auto base_foot_pos = standing_positions.at(index);

        // Compute this leg's local phase within its cycle
        double local_phase = std::fmod(phase_ + leg_info.phase_offset, 2.0 * M_PI);
        if (local_phase < 0.0) local_phase += 2.0 * M_PI;

        double step = 0.0;
        double lift = 0.0;

        if (local_phase < kTransferPhase) {
            // Transfer phase: leg in air, sweeping forward
            double t = local_phase / kTransferPhase;  // normalized 0..1
            step = -std::cos(t * M_PI) * params_.gait_step_length;
            lift = std::sin(t * M_PI) * params_.leg_lift_height;
        } else {
            // Support phase: leg on ground, sliding backward
            double t = (local_phase - kTransferPhase) / kSupportPhase;  // normalized 0..1
            step = std::cos(t * M_PI) * params_.gait_step_length;
            lift = 0.0;
        }

        // Linear displacement
        double delta_x = norm_x * step;
        double delta_y = norm_y * step;

        // Rotational displacement
        double leg_vec_x = base_foot_pos.x;
        double leg_vec_y = base_foot_pos.y;
        double len = std::sqrt(leg_vec_x * leg_vec_x + leg_vec_y * leg_vec_y);

        double rot_x = 0.0;
        double rot_y = 0.0;

        if (len > 1e-6) {
            double dir_x = -leg_vec_y / len;
            double dir_y = leg_vec_x / len;
            rot_x = dir_x * step * norm_rot;
            rot_y = dir_y * step * norm_rot;
        }

        CPosition target;
        target.x = base_foot_pos.x + delta_x + rot_x;
        target.y = base_foot_pos.y + delta_y + rot_y;
        target.z = base_foot_pos.z + lift;

        target_positions_[index] = target;
    }

    kinematics_->moveBody(target_positions_, body);

    COrientation head_request;
    head_request.yaw_deg = params_.head_amplitude_yaw_deg * std::sin(phase_);
    kinematics_->setHead(head_request);
    return true;
}

void CWaveGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::StopPending;
    }
}

void CWaveGait::cancelStop() {
    if (state_ == EGaitState::StopPending) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
