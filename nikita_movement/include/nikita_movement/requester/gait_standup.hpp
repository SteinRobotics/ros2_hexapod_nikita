#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nikita_utils/linear_interpolation.hpp"
#include "requester/gaits_parameter.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace nikita_movement {

class CStandUpGait : public IGait {
   public:
    CStandUpGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                 Parameters::StandUp& params);
    ~CStandUpGait() override = default;

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
    Parameters::StandUp params_;

    EGaitState state_ = EGaitState::Stopped;
    std::map<ELegIndex, CPosition> origin_positions_;

    double phase_ = 0.0;
};

}  // namespace nikita_movement
