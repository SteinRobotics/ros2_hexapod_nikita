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
#include "nikita_utils/filters.hpp"
#include "nikita_utils/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"
// Gait interfaces and implementations
// Forward declarations to avoid heavy includes here
namespace nikita_movement {
class IGait;
class CTripodGait;
class CRippleGait;
class CBodyRollGait;
class CLegRollGait;
}  // namespace nikita_movement

class CGaitController {
   public:
    CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics);
    // Out-of-line destructor to ensure IGait is a complete type at destruction
    virtual ~CGaitController();

    // Simple gait selection API
    enum class EGaitType { Tripod, Ripple, BodyRoll, LegRoll };
    void setGait(EGaitType type);
    EGaitType currentGait() const {
        return active_type_;
    }

    // Delegate update to selected gait
    void updateSelectedGait(const geometry_msgs::msg::Twist& velocity,
                            CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    void requestStopSelectedGait();
    void cancelStopSelectedGait();

    void resetPhase();
    void liftLegsTripodGroup(bool isFirstTripod = true);
    void updateTripodGait(const geometry_msgs::msg::Twist& velocity,
                          CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    void updateBodyRoll();

    void initLegRoll();
    void updateLegRoll();

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
    bool isStartPhaseDone_ = false;
    bool isLeaveCycleRequested_ = false;
    ELegIndex activeLegIndex_ = ELegIndex::RightFront;

    geometry_msgs::msg::Twist velocity_{geometry_msgs::msg::Twist()};

    // Owned gait implementations
    std::unique_ptr<nikita_movement::IGait> gait_tripod_;
    std::unique_ptr<nikita_movement::IGait> gait_ripple_;
    std::unique_ptr<nikita_movement::IGait> gait_bodyroll_;
    std::unique_ptr<nikita_movement::IGait> gait_legroll_;

    // Currently active gait (non-owning)
    nikita_movement::IGait* active_gait_ = nullptr;
    EGaitType active_type_ = EGaitType::Tripod;
};
