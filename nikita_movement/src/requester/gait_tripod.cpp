#include "requester/gait_tripod.hpp"

#include "requester/kinematics.hpp"

namespace nikita_movement {

constexpr double TIME_TO_WAIT_BEFORE_STOP_SEC = 3.0;

CTripodGait::CTripodGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                         double legLiftHeight, double gaitStepLength, double factorVelocityToGaitCycleTime)
    : node_(std::move(node)),
      kinematics_(std::move(kinematics)),
      legLiftHeight_(legLiftHeight),
      gaitStepLength_(gaitStepLength),
      factorVelocityToGaitCycleTime_(factorVelocityToGaitCycleTime) {
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
        } else if (no_velocity_timer_.haveSecondsElapsed(TIME_TO_WAIT_BEFORE_STOP_SEC)) {
            requestStop();
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
    double deltaPhase = factorVelocityToGaitCycleTime_ * combined_mag;
    phase_ += deltaPhase;

    // this if condition is only relevant for the first movement of the FirstTripod legs and only for 0 < phase_ < M_PI_4
    // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again
    if (state_ == EGaitState::Starting && phase_ > M_PI_4) {
        state_ = EGaitState::Running;
    }
    if (state_ == EGaitState::StopPending && utils::areSinCosValuesEqual(phase_, deltaPhase)) {
        state_ = EGaitState::Stopping;
    }
    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, deltaPhase)) {
        state_ = EGaitState::Stopped;
        return false;
    }

    // RCLCPP_INFO(node_->get_logger(), "CGaitController::updateTripodGait: phase: %.4f", phase_);

    std::map<ELegIndex, CPosition> targetPositions;

    for (auto& [index, leg] : kinematics_->getLegs()) {
        bool isFirstTripodActive = std::ranges::find(groupFirstTripod_, index) != groupFirstTripod_.end();

        double phaseOffset = isFirstTripodActive ? 0.0 : M_PI;
        double phaseWithOffset = phase_ + phaseOffset + M_PI_2;

        double step = gaitStepLength_ * cos(phaseWithOffset);

        double lift = 0.0;
        if (state_ == EGaitState::Running || state_ == EGaitState::StopPending) {
            lift = legLiftHeight_ * std::max(0.0, sin(phaseWithOffset));
        } else if (state_ == EGaitState::Starting || state_ == EGaitState::Stopping) {
            // this if condition is only relevant for the first movement of the FirstTripod legs and only for 0 < phase_ < M_PI_4
            // phase_ == M_PI_4 is reached when the leg is moving upwards and the normal cycle goes downwards again

            // Starting is using the first half of the sine (0 to M_PI_4) to lift the leg up, target is 1
            // Stopping is using the second half of the sine wave (M_PI_4 to M_PI_2), to bring the leg down, target is 0
            lift = legLiftHeight_ * std::max(0.0, sin(phase_));
        }

        const auto baseFootPos = kinematics_->getLegsStandingPositions().at(index);

        // linear
        double deltaX = norm_x * step;
        double deltaY = norm_y * step;

        // rotation
        double legVecX = baseFootPos.x;
        double legVecY = baseFootPos.y;
        double len = std::sqrt(legVecX * legVecX + legVecY * legVecY);

        double rotX = 0.0;
        double rotY = 0.0;

        if (len > 1e-6) {
            double dirX = -legVecY / len;
            double dirY = legVecX / len;

            rotX = dirX * step * norm_rot;
            rotY = dirY * step * norm_rot;
        }

        CPosition target;
        target.x = baseFootPos.x + deltaX + rotX;
        target.y = baseFootPos.y + deltaY + rotY;
        target.z = baseFootPos.z + lift;

        targetPositions[index] = target;
    }

    kinematics_->moveBody(targetPositions, body);

    constexpr double MAX_HEAD_YAW_AMPLITUDE_DEG = 15.0;
    kinematics_->getHead().yaw_deg = MAX_HEAD_YAW_AMPLITUDE_DEG * std::sin(phase_);
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
