#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "requester/igaits.hpp"
#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

class CHighFiveGait : public IGait {
   public:
    CHighFiveGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    ~CHighFiveGait() override = default;

    void start() override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    enum class EPhase { Idle, Raising, Holding, Lowering, Finished };

    void applyInterpolatedPose(double alpha);
    void transitionToLowering();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;

    EGaitState state_ = EGaitState::Stopped;
    EPhase phase_ = EPhase::Idle;

    CLegAngles initial_leg_angles_{};
    CLegAngles target_leg_angles_{20.0, 50.0, 60.0};
    CHead initial_head_{};
    double target_head_yaw_deg_ = 0.0;
    double target_head_pitch_deg_ = -20.0;

    double phase_progress_ = 0.0;
    int hold_iterations_remaining_ = 0;
};

}  // namespace nikita_movement
