#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nikita_utils/geometry.hpp"
#include "requester/igaits.hpp"
#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

class CGaitLegWave : public IGait {
   public:
    CGaitLegWave(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    ~CGaitLegWave() override = default;

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

    double kLegLiftHeight_ = double(0);

    double phase_ = double(0);
    ELegIndex active_leg_index_ = ELegIndex::RightFront;

    std::vector<ELegIndex> leg_order_ = {ELegIndex::RightFront, ELegIndex::RightMid, ELegIndex::RightBack,
                                         ELegIndex::LeftFront,  ELegIndex::LeftMid,  ELegIndex::LeftBack};
};

}  // namespace nikita_movement
