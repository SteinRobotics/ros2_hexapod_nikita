#pragma once

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nikita_interfaces/msg/movement_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"

namespace nikita_movement {

enum class EGaitState;

class CGaitLook : public IGait {
   public:
    CGaitLook(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;

    double amplitude_head_deg_ = 0.0;
    double amplitude_body_deg_ = 0.0;
    double delta_phase_ = 0.0;
    double phase_ = 0.0;

    EGaitState state_ = EGaitState::Stopped;
};

}  // namespace nikita_movement
