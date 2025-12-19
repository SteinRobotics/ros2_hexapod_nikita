#pragma once

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nikita_interfaces/msg/movement_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"

namespace nikita_movement {

class CGaitWatch : public IGait {
   public:
    CGaitWatch(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
               Parameters::Watch& params);
    ~CGaitWatch() override = default;

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
    Parameters::Watch params_;

    double kHeadMaxYaw_ = double(0);
    double kBodyMaxYaw_ = double(0);

    double amplitude_head_deg_ = 0.0;
    double amplitude_body_deg_ = 0.0;
    double delta_phase_ = 0.0;
    double phase_ = 0.0;

    EGaitState state_ = EGaitState::Stopped;
};

}  // namespace nikita_movement
