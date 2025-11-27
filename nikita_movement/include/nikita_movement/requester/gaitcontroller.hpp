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
#include "nikita_interfaces/msg/movement_request.hpp"
#include "nikita_utils/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/gait_bodyroll.hpp"
#include "requester/gait_clap.hpp"
#include "requester/gait_highfive.hpp"
#include "requester/gait_laydown.hpp"
#include "requester/gait_legwave.hpp"
#include "requester/gait_ripple.hpp"
#include "requester/gait_standup.hpp"
#include "requester/gait_tripod.hpp"
#include "requester/gait_waiting.hpp"
#include "requester/gait_watch.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

class CGaitController {
   public:
    CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    virtual ~CGaitController();

    using MovementRequestType = nikita_interfaces::msg::MovementRequest::_type_type;

    void setGait(nikita_interfaces::msg::MovementRequest request);
    MovementRequestType currentGait() const {
        return active_type_;
    }

    bool updateSelectedGait(const geometry_msgs::msg::Twist& velocity,
                            CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    void requestStopSelectedGait();

   private:
    void switchGait(MovementRequestType type);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;

    std::unique_ptr<nikita_movement::IGait> gait_tripod_;
    std::unique_ptr<nikita_movement::IGait> gait_ripple_;
    std::unique_ptr<nikita_movement::IGait> gait_bodyroll_;
    std::unique_ptr<nikita_movement::IGait> gait_legwave_;
    std::unique_ptr<nikita_movement::IGait> gait_waiting_;
    std::unique_ptr<nikita_movement::IGait> gait_watch_;
    std::unique_ptr<nikita_movement::IGait> gait_standup_;
    std::unique_ptr<nikita_movement::IGait> gait_laydown_;
    std::unique_ptr<nikita_movement::IGait> gait_highfive_;
    std::unique_ptr<nikita_movement::IGait> gait_clap_;

    double kFactorVelocityToGaitCycleTime = double(0);
    double kLegLiftHeight = double(0);
    double kGaitStepLength = double(0);

    nikita_movement::IGait* active_gait_ = nullptr;
    MovementRequestType pending_type_ = nikita_interfaces::msg::MovementRequest::NO_REQUEST;
    MovementRequestType active_type_ = nikita_interfaces::msg::MovementRequest::MOVE_TRIPOD;
};
