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
#include "nikita_utils/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/gait_bodyroll.hpp"
#include "requester/gait_legwave.hpp"
#include "requester/gait_ripple.hpp"
#include "requester/gait_tripod.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

class CGaitController {
   public:
    CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    virtual ~CGaitController();

    enum class EGaitType { Tripod, Ripple, BodyRoll, LegWave, Waiting };
    void setGait(EGaitType type);
    EGaitType currentGait() const {
        return active_type_;
    }

    bool updateSelectedGait(const geometry_msgs::msg::Twist& velocity,
                            CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    void requestStopSelectedGait();

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;

    std::unique_ptr<nikita_movement::IGait> gait_tripod_;
    std::unique_ptr<nikita_movement::IGait> gait_ripple_;
    std::unique_ptr<nikita_movement::IGait> gait_bodyroll_;
    std::unique_ptr<nikita_movement::IGait> gait_legwave_;

    double kFactorVelocityToGaitCycleTime = double(0);
    double kLegLiftHeight = double(0);
    double kGaitStepLength = double(0);

    nikita_movement::IGait* active_gait_ = nullptr;
    EGaitType active_type_ = EGaitType::Tripod;
};
