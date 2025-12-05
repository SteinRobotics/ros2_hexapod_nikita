#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nikita_utils/geometry.hpp"
#include "requester/igaits.hpp"
#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

class CWatchGait : public IGait {
   public:
    CWatchGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    ~CWatchGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    enum class EPhase { Idle, ToLeft, ToRight, ReturnToOrigin, Finished };

    struct PoseTarget {
        double headYaw;
        double headPitch;
        double bodyRoll;
        double bodyPitch;
        double bodyYaw;
    };

    void applyInterpolatedPose(const PoseTarget& from, const PoseTarget& to, double alpha);
    void transitionToPhase(EPhase next_phase, const PoseTarget& from, const PoseTarget& to);
    PoseTarget composeTarget(double headYawDelta, double headPitchDelta, double bodyYawDelta) const;
    PoseTarget captureCurrentTarget() const;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;

    EGaitState state_ = EGaitState::Stopped;
    EPhase phase_ = EPhase::Idle;
    double phase_progress_ = 0.0;

    CHead initial_head_{};
    CPose initial_body_pose_{};
    std::map<ELegIndex, CPosition> base_foot_positions_{};

    PoseTarget start_target_{};
    PoseTarget end_target_{};
    PoseTarget initial_target_{};
    PoseTarget left_target_{};
    PoseTarget right_target_{};
};

}  // namespace nikita_movement
