#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace nikita_movement {

class CNeutralGait : public IGait {
   public:
    CNeutralGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    ~CNeutralGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    void applyNeutralPose(double progress);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;

    EGaitState state_ = EGaitState::Stopped;
    std::map<ELegIndex, CLegAngles> origin_leg_angles_;
    std::map<ELegIndex, CLegAngles> target_leg_angles_;
    CHead origin_head_;
    CHead target_head_{0.0, 0.0};
    double phase_ = 0.0;
};

}  // namespace nikita_movement
