#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nikita_utils/linear_interpolation.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace nikita_movement {

class CGaitBodyPose : public IGait {
   public:
    CGaitBodyPose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                  Parameters::BodyPose& params);
    ~CGaitBodyPose() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body,
                const COrientation& head) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::BodyPose params_;
    EGaitState state_ = EGaitState::Stopped;

    CPose body_origin_ = CPose();
    CHead origin_head_ = CHead();

    double duration_s_ = 0.0;
    double phase_increment_ = 0.1;
    double phase_ = 0.0;
};

}  // namespace nikita_movement
