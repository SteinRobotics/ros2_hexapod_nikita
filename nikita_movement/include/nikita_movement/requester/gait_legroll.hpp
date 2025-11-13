#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "requester/igaits.hpp"
#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

class CLegRollGait : public IGait {
   public:
    CLegRollGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    ~CLegRollGait() override = default;

    void start() override;
    void update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    EGaitState state_ = EGaitState::Stopped;

    double LEG_LIFT_HEIGHT = double(0);

    double phase_ = double(0);
    ELegIndex activeLegIndex_ = ELegIndex::RightFront;
};

}  // namespace nikita_movement