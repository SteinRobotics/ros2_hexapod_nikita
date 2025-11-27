#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "requester/igaits.hpp"
#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

class CClapGait : public IGait {
   public:
    CClapGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    ~CClapGait() override = default;

    void start() override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    enum class EPhase {
        Idle,
        ShiftingBack,
        LiftRightBack,
        LowerRightBack,
        LiftLeftBack,
        LowerLeftBack,
        LiftFrontLegs,
        ClapClosing,
        ClapOpening,
        LowerFrontLegs,
        ShiftingForward,
        Finished
    };

    void applyBodyShift(double alpha);
    void applyBackLegLift(ELegIndex leg, double alpha);
    void applyFrontLegsLift(double alpha);
    void applyFrontLegsClap(double alpha, bool closing);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;

    EGaitState state_ = EGaitState::Stopped;
    EPhase phase_ = EPhase::Idle;

    // Store initial positions
    std::map<ELegIndex, CPosition> initial_foot_positions_;
    CPose initial_body_pose_;

    double phase_progress_ = 0.0;
    int clap_iterations_remaining_ = 0;
};

}  // namespace nikita_movement
