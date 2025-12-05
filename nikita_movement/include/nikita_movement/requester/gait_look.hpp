#pragma once

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"

namespace nikita_movement {

// EGaitState is declared in requester/igaits.hpp

class CGaitHeadLook {
   public:
    CGaitHeadLook(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);

    void start(double duration_s, uint8_t direction);
    bool update(const geometry_msgs::msg::Twist& /*velocity*/, const CPose& /*body*/);
    void requestStop();
    void cancelStop();
    EGaitState state() const {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;

    double kHeadMaxYaw_ = double(0);
    double kBodyMaxYaw_ = double(0);

    double amplitude_deg_;
    double speed_;
    double phase_ = 0.0;

    EGaitState state_ = EGaitState::Stopped;
};

}  // namespace nikita_movement
