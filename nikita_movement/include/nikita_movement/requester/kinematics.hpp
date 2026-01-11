/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "nikita_interfaces/msg/pose.hpp"
#include "nikita_utils/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/types.hpp"

namespace nikita_movement {

class CKinematics {
   public:
    explicit CKinematics(std::shared_ptr<rclcpp::Node> node);
    ~CKinematics() = default;

    void setSingleFeet(const ELegIndex index, const CPosition& targetFeetPos);
    void setLegAngles(const ELegIndex index, const CLegAngles& angles);
    void moveBody(const std::map<ELegIndex, CPosition>& footTargets,
                  const CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    void moveBody(const CPose body);

    // New version of moveBody which uses the segment-wise IK solver (solveIKSegmentwise)
    // to compute joint states per leg from world/body-frame foot targets.
    // void moveBodyNew(const std::map<ELegIndex, CPosition>& footTargets,
    //                  const CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    void setHead(double yaw_deg, double pitch_deg);
    void setHead(COrientation head);

    std::map<ELegIndex, CPosition> getLegsPositions() const;
    std::map<ELegIndex, CPosition> getLegsStandingPositions() const;
    std::map<ELegIndex, CPosition> getLegsLayDownPositions() const;

    std::map<ELegIndex, CLeg>& getLegs();
    CLegAngles& getAngles(ELegIndex index);
    std::map<ELegIndex, CLegAngles> getLegsAngles();

    COrientation& getHead() {
        return head_;
    };

    CPose& getBody() {
        return body_;
    };

    // Expose the complete body (segment-wise) for testing/inspection
    // const CBody& getCompleteBody() const;

   private:
    void initializeLegs(const std::map<ELegIndex, CPosition>& footTargets, const CPose body,
                        std::map<ELegIndex, CLeg>& legs);

    void logLegsPositions(std::map<ELegIndex, CLeg>& legs);
    void logLegPosition(const ELegIndex index, const CLeg& leg);
    void logHeadPosition();
    void calcLegInverseKinematics(const CPosition& targetFeetPos, CLeg& leg, const ELegIndex& legIndex);
    void calcLegForwardKinematics(const CLegAngles target, CLeg& leg);
    CPosition rotate(const CPosition& point, const COrientation& rot);

    // Publish the current joint angles (coxa/femur/tibia for each leg) as a JointState
    // void publishJointStates();

    std::shared_ptr<rclcpp::Node> node_;

    // Parameters
    const double COXA_LENGTH;
    const double COXA_HEIGHT;
    const double FEMUR_LENGTH;
    const double TIBIA_LENGTH;
    const double sq_femur_length_;
    const double sq_tibia_length_;

    std::map<ELegIndex, CLeg> legs_;          // current values
    std::map<ELegIndex, CLeg> legsStanding_;  // change to shared pointer and make const
    std::map<ELegIndex, CLeg> legsLayDown_;   // change to shared pointer and make const
    std::map<ELegIndex, CBodyCenterOffset> bodyCenterOffsets_;

    CPose body_ = {};
    COrientation head_ = {};
};

}  // namespace nikita_movement