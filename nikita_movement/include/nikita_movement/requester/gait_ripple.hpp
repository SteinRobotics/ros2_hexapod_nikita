#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "requester/igaits.hpp"
#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

class CRippleGait : public IGait {
   public:
    CRippleGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    ~CRippleGait() override = default;

    void start() override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    EGaitState state_ = EGaitState::Stopped;
};

}  // namespace nikita_movement
