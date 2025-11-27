#include "requester/gait_legwave.hpp"

#include <magic_enum.hpp>

#include "requester/kinematics.hpp"

namespace nikita_movement {

void CGaitLegWave::start() {
    state_ = EGaitState::Running;
    phase_ = 0.0;
    active_leg_index_ = ELegIndex::RightFront;
}

bool CGaitLegWave::update(const geometry_msgs::msg::Twist& velocity, const CPose& /*body*/) {
    if (state_ == EGaitState::Stopped) {
        return false;
    }
    // TODO: calc velocity_mag to delta_phase
    constexpr double delta_phase = 0.1;
    phase_ += delta_phase;

    if (state_ == EGaitState::Stopping && utils::isSinValueNearZero(phase_, delta_phase)) {
        state_ = EGaitState::Stopped;
        return false;
    }

    // TODO better use kinematics_->getLegsPositions()
    const auto base_foot_pos = kinematics_->getLegsStandingPositions();

    if (phase_ >= M_PI) {
        // reset last leg to neutral position
        kinematics_->setSingleFeet(active_leg_index_, base_foot_pos.at(active_leg_index_));

        // advance to the next leg
        int forward = true;
        if (velocity.linear.x < 0.0) {
            forward = false;
        }
        active_leg_index_ = leg_order_[(std::find(leg_order_.begin(), leg_order_.end(), active_leg_index_) -
                                        leg_order_.begin() + (1 * forward)) %
                                       leg_order_.size()];
        phase_ = 0.0;
    }

    auto target_position = base_foot_pos.at(active_leg_index_);
    target_position.z = base_foot_pos.at(active_leg_index_).z + kLegLiftHeight * std::sin(phase_);

    kinematics_->setSingleFeet(active_leg_index_, target_position);
    return true;
}

void CGaitLegWave::requestStop() {
    if (state_ == EGaitState::Running) {
        state_ = EGaitState::Stopping;
    }
}

void CGaitLegWave::cancelStop() {
    if (state_ == EGaitState::Stopping) {
        state_ = EGaitState::Running;
    }
}

}  // namespace nikita_movement
