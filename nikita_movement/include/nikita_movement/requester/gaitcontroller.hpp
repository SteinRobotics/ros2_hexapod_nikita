/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"
#include "requester/lowpassfilter.hpp"

class CGaitController {
   public:
    CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    ~CGaitController() = default;

    void liftLegsTripodGroup(bool isFirstTripod = true);
    void updateCombinedTripodGait(const geometry_msgs::msg::Twist& velocity,
                                  CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    void setPhaseNeutral();

    double LEG_LIFT_HEIGHT = double(0);
    double GAIT_STEP_LENGTH = double(0);
    double FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME = double(0);

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    bool useGroup1_ = true;

    const std::vector<ELegIndex> groupFirstTripod_ = {ELegIndex::LeftFront, ELegIndex::RightMid,
                                                      ELegIndex::LeftBack};
    const std::vector<ELegIndex> groupSecondTripod_ = {ELegIndex::RightFront, ELegIndex::LeftMid,
                                                       ELegIndex::RightBack};

    double phase_ = double(0);
    geometry_msgs::msg::Twist velocity_{geometry_msgs::msg::Twist()};
};
