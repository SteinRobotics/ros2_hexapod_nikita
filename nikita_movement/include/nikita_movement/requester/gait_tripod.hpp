#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nikita_utils/filters.hpp"
#include "nikita_utils/geometry.hpp"
#include "nikita_utils/msg_twist.hpp"
#include "nikita_utils/simpletimer.hpp"
#include "requester/igaits.hpp"
#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

class CTripodGait : public IGait {
   public:
    CTripodGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                double legLiftHeight, double gaitStepLength, double factorVelocityToGaitCycleTime);
    ~CTripodGait() override = default;

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
    double legLiftHeight_ = double(0);
    double gaitStepLength_ = double(0);
    double factorVelocityToGaitCycleTime_ = double(0);
    EGaitState state_ = EGaitState::Stopped;

    bool useGroup1_ = true;

    const std::vector<ELegIndex> groupFirstTripod_ = {ELegIndex::LeftFront, ELegIndex::RightMid,
                                                      ELegIndex::LeftBack};
    const std::vector<ELegIndex> groupSecondTripod_ = {ELegIndex::RightFront, ELegIndex::LeftMid,
                                                       ELegIndex::RightBack};

    double phase_ = double(0);
    bool isStartPhaseDone_ = false;
    bool isLeaveCycleRequested_ = false;
    ELegIndex activeLegIndex_ = ELegIndex::RightFront;

    CSimpleTimer no_velocity_timer_;

    geometry_msgs::msg::Twist velocity_{geometry_msgs::msg::Twist()};
};

}  // namespace nikita_movement
