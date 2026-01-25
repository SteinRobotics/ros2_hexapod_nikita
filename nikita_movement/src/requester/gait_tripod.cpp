#include "requester/gait_tripod.hpp"

namespace nikita_movement {

constexpr double kTimeToWaitBeforeStopSec = 3.0;

CTripodGait::CTripodGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                         Parameters::Tripod& params)
    : node_(node), kinematics_(kinematics), params_(params) {
    no_velocity_timer_.stop();
    body_old_ = CPose();
    target_positions_ = kinematics_->getLegsStandingPositions();
}

void CTripodGait::start(double /*duration_s*/, uint8_t /*direction*/) {
    state_ = EGaitState::Starting;
    phase_ = 0.0;
}

bool CTripodGait::update(const geometry_msgs::msg::Twist& velocity, const CPose& body,
                         const COrientation& /*head*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }

    if (utils::isTwistZero(velocity) && state_ == EGaitState::Running && body != body_old_) {
        // Body position changed while standing still; update body position without moving legs
        const auto base_foot_pos = kinematics_->getLegsStandingPositions();
        kinematics_->moveBody(base_foot_pos, body);
        body_old_ = body;
        return true;
    }

    if (utils::isTwistZero(velocity) && state_ == EGaitState::Running) {
        if (!no_velocity_timer_.isRunning()) {
            no_velocity_timer_.start();
        } else if (no_velocity_timer_.haveSecondsElapsed(kTimeToWaitBeforeStopSec)) {
            requestStop();
            no_velocity_timer_.stop();
        }
        return false;
    }

    // filter and store the last non-zero velocity, zero velocities will be ignored and the old velocity is kept
    if (!utils::isTwistZero(velocity)) {
        const double alpha = 0.01;  // Adjust alpha for filtering strength (0.0 to 1.0)
        velocity_ = utils::limitChangeRateUpTwist(velocity_, velocity, alpha);
    }
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CGaitController::requestMove: filtered velocity: "
    //                                             << velocity_.linear.x << ", " << velocity_.linear.y << ", "
    //                                             << velocity_.angular.z);

    double linear_x = velocity_.linear.x;
    double linear_y = velocity_.linear.y;
    double angular_z = velocity_.angular.z;

    constexpr double ROTATION_WEIGHT = 0.7;

    double combined_mag =
        std::sqrt((linear_x * linear_x) + (linear_y * linear_y) + (ROTATION_WEIGHT * angular_z * angular_z));

    // Avoid division by zero
    if (combined_mag < 1e-6) return false;

    double norm_x = linear_x / combined_mag;
    double norm_y = linear_y / combined_mag;
    double norm_rot = angular_z / combined_mag;

    // the velocity is taken into account to calculate the gait cycle time with combined_mag
    double delta_phase = params_.factor_velocity_to_gait_cycle_time * combined_mag;
    phase_ += delta_phase;
    phase_ = std::fmod(phase_, 2.0 * M_PI);

    // this if condition is only relevant for the first movement of the FirstTripod legs and only for 0 < phase_ < M_PI_4
    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (state_ == EGaitState::Starting && phase_ > M_PI_4) {
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::StopPending && utils::isValueNear((7.0 / 8.0) * M_PI, phase_, delta_phase)) {
        RCLCPP_INFO(node_->get_logger(), "CTripodGait::update: Transitioning to Stopping state, phase_: %.2f",
                    phase_);
        state_ = EGaitState::Stopping;
        is_first_tripod_tiggered_stopping_ = true;
    }
    if (state_ == EGaitState::StopPending && utils::isValueNear((3.0 / 4.0) * M_PI, phase_, delta_phase)) {
        RCLCPP_INFO(node_->get_logger(), "CTripodGait::update: Transitioning to Stopping state, phase_: %.2f",
                    phase_);
        state_ = EGaitState::Stopping;
        is_first_tripod_tiggered_stopping_ = false;
    }
    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, delta_phase)) {
        RCLCPP_INFO(node_->get_logger(), "CTripodGait::update: Transitioning to Stopped state, phase_: %.2f",
                    phase_);
        phase_ = 0.0;
        state_ = EGaitState::Stopped;
        kinematics_->moveBody(kinematics_->getLegsStandingPositions(), body);
        kinematics_->setHead(COrientation(0.0, 0.0, 0.0));
        return true;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "CTripodGait::update: state_: " << magic_enum::enum_name(state_));
    RCLCPP_INFO(node_->get_logger(), "CTripodGait::update: phase_: %.2f, delta_phase: %.2f", phase_,
                delta_phase);

    for (auto& [index, leg] : kinematics_->getLegs()) {
        bool is_first_tripod_active =
            std::ranges::find(group_first_tripod_, index) != group_first_tripod_.end();

        double phase_offset = is_first_tripod_active ? 0.0 : M_PI;
        double phase_with_offset = phase_ + phase_offset;

        double step = params_.gait_step_length * sin(phase_with_offset);

        double lift = 0.0;

        auto lift_following_cos = params_.leg_lift_height * pow(std::max(0.0, cos(phase_with_offset)), 2);
        auto lift_following_sin = params_.leg_lift_height * pow(std::max(0.0, sin(phase_with_offset)), 2);

        // default lift follows the cosine wave
        lift = lift_following_cos;

        if (state_ == EGaitState::Starting && is_first_tripod_active) {
            lift = lift_following_sin;
        } else if (state_ == EGaitState::Stopping && is_first_tripod_active &&
                   is_first_tripod_tiggered_stopping_) {
            lift = lift_following_sin;
        } else if (state_ == EGaitState::Stopping && !is_first_tripod_active &&
                   !is_first_tripod_tiggered_stopping_) {
            lift = lift_following_sin;
        }

        const auto base_foot_pos = kinematics_->getLegsStandingPositions().at(index);

        // linear
        double delta_x = norm_x * step;
        double delta_y = norm_y * step;

        // rotation
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

void CTripodGait::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::StopPending;
    }
}

void CTripodGait::cancelStop() {
    // if the state is not in state StopPending, cancel the transition to Stop is not possible
    if (state_ == EGaitState::StopPending) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
