#include "requester/gait_tripod.hpp"

#include "requester/kinematics.hpp"

namespace nikita_movement {

constexpr double kTimeToWaitBeforeStopSec = 3.0;

CTripodGait::CTripodGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(node), kinematics_(kinematics) {
    kFactorVelocityToGaitCycleTime_ =
        node_->declare_parameter<double>("FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME", rclcpp::PARAMETER_DOUBLE);
    kGaitStepLength_ = node_->declare_parameter<double>("GAIT_STEP_LENGTH", rclcpp::PARAMETER_DOUBLE);
    kLegLiftHeight_ = node_->declare_parameter<double>("LEG_LIFT_HEIGHT", rclcpp::PARAMETER_DOUBLE);
    kHeadMaxYawAmplitude_ = node_->declare_parameter<double>("HEAD_MAX_YAW_TRIPOD", rclcpp::PARAMETER_DOUBLE);
    no_velocity_timer_.stop();
}

void CTripodGait::start() {
    state_ = EGaitState::Starting;
    phase_ = 0.0;
}

bool CTripodGait::update(const geometry_msgs::msg::Twist& velocity, const CPose& body) {
    if (state_ == EGaitState::Stopped) {
        return false;
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
        const double alpha = 0.5;  // Adjust alpha for filtering strength (0.0 to 1.0)
        velocity_ = utils::lowPassFilterTwist(velocity_, velocity, alpha);
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
    double delta_phase = kFactorVelocityToGaitCycleTime_ * combined_mag;
    phase_ += delta_phase;

    // this if condition is only relevant for the first movement of the FirstTripod legs and only for 0 < phase_ < M_PI_4
    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (state_ == EGaitState::Starting && phase_ > M_PI_4) {
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::StopPending && utils::areSinCosValuesEqual(phase_, delta_phase)) {
        RCLCPP_INFO(node_->get_logger(), "CTripodGait::update: Transitioning to Stopping state, phase_: %.4f",
                    phase_);
        state_ = EGaitState::Stopping;
    }
    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, delta_phase)) {
        RCLCPP_INFO(node_->get_logger(), "CTripodGait::update: Transitioning to Stopped state, phase_: %.4f",
                    phase_);
        phase_ = 0.0;
        state_ = EGaitState::Stopped;
        // run one last update
    }

    std::map<ELegIndex, CPosition> target_positions;

    for (auto& [index, leg] : kinematics_->getLegs()) {
        bool is_first_tripod_active =
            std::ranges::find(group_first_tripod_, index) != group_first_tripod_.end();

        double phase_offset = is_first_tripod_active ? 0.0 : M_PI;
        double phase_with_offset = phase_ + phase_offset + M_PI_2;

        double step = kGaitStepLength_ * cos(phase_with_offset);

        double lift = 0.0;
        if (state_ == EGaitState::Running || state_ == EGaitState::StopPending) {
            lift = kLegLiftHeight_ * std::max(0.0, sin(phase_with_offset));
        } else if (state_ == EGaitState::Starting || state_ == EGaitState::Stopping) {
            // this if condition is only relevant for the first movement of the FirstTripod legs and only for 0 < phase_ < M_PI_4
            // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again

            // Starting is using the first half of the sine (0 to M_PI_4) to lift the leg up, target is 1
            // Stopping is using the second half of the sine wave (M_PI_4 to M_PI_2), to bring the leg down, target is 0
            lift = kLegLiftHeight_ * std::max(0.0, sin(phase_));
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

        target_positions[index] = target;
    }

    kinematics_->moveBody(target_positions, body);

    kinematics_->getHead().yaw_deg = kHeadMaxYawAmplitude_ * std::sin(phase_);
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
