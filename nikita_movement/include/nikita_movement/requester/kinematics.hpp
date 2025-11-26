/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "nikita_interfaces/msg/pose.hpp"
#include "nikita_utils/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "requester/types.hpp"

class CKinematics {
   public:
    CKinematics(std::shared_ptr<rclcpp::Node> node,
                std::shared_ptr<CActionPackagesParser> actionPackagesParser);
    ~CKinematics() = default;

    void setSingleFeet(const ELegIndex index, const CPosition& targetFeetPos);
    void setLegAngles(const ELegIndex index, const CLegAngles& angles);
    void moveBody(const std::map<ELegIndex, CPosition>& footTargets,
                  const CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    // New version of moveBody which uses the segment-wise IK solver (solveIKSegmentwise)
    // to compute joint states per leg from world/body-frame foot targets.
    void moveBodyNew(const std::map<ELegIndex, CPosition>& footTargets,
                     const CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    void setHead(double yaw_deg, double pitch_deg);
    void setHead(CHead head);

    std::map<ELegIndex, CPosition> getLegsPositions() const;
    std::map<ELegIndex, CPosition> getLegsStandingPositions() const;
    std::map<ELegIndex, CPosition> getLegsLayDownPositions() const;

    std::map<ELegIndex, CLeg>& getLegs();
    CLegAngles& getAngles(ELegIndex index);
    std::map<ELegIndex, CLegAngles> getLegsAngles();

    CHead& getHead() {
        return head_;
    };

    CPose& getBody() {
        return body_;
    };

    // Expose the complete body (segment-wise) for testing/inspection
    const CBody& getCompleteBody() const;

    // Parameters
    std::vector<std::string> LEG_NAMES;

    double COXA_LENGTH = double(0);
    double COXA_HEIGHT = double(0);
    double FEMUR_LENGTH = double(0);
    double TIBIA_LENGTH = double(0);

    std::vector<double> CENTER_TO_COXA_X;
    std::vector<double> CENTER_TO_COXA_Y;
    std::vector<double> OFFSET_COXA_ANGLE_DEG;

    double BODY_MAX_ROLL = double(0);
    double BODY_MAX_PITCH = double(0);
    double BODY_MAX_YAW = double(0);
    double HEAD_MAX_YAW = double(0);
    double HEAD_MAX_PITCH = double(0);

    // NOTE: segment-wise kinematics helpers are declared as free functions below

   private:
    void intializeLegs(std::map<ELegIndex, CLeg>& legs, std::vector<double>& posX, std::vector<double>& posY,
                       std::vector<double>& posZ);

    void initializeLegsNew(const std::map<ELegIndex, CPosition>& footTargets, const CPose body,
                           std::map<ELegIndex, CLeg>& legs);

    void logLegsPositions(std::map<ELegIndex, CLeg>& legs);
    void logLegPosition(const ELegIndex index, const CLeg& leg);
    void logHeadPosition();
    void calcLegInverseKinematics(const CPosition& targetFeetPos, CLeg& leg, const ELegIndex& legIndex);
    void calcLegForwardKinematics(const CLegAngles target, CLeg& leg);
    CPosition rotate(const CPosition& point, const COrientation& rot);

    // Publish the current joint angles (coxa/femur/tibia for each leg) as a JointState
    void publishJointStates();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPackagesParser> actionPackagesParser_;

    std::map<ELegIndex, CLeg> legs_;          // current values
    std::map<ELegIndex, CLeg> legsStanding_;  // change to shared pointer and make const
    std::map<ELegIndex, CLeg> legsLayDown_;   // change to shared pointer and make const
    std::map<ELegIndex, CBodyCenterOffset> bodyCenterOffsets_;

    CPose body_ = {};
    CHead head_ = {};

    CBody complete_body_ = {};  // rename later to body_

    double sqFemurLength_ = 0.0;
    double sqTibiaLength_ = 0.0;
    // Publisher for joint states (coxa/femur/tibia for each leg)
    // Use the non-templated PublisherBase pointer here to avoid requiring
    // the sensor_msgs header in this public header file. The concrete
    // publisher is created in the implementation file.
    rclcpp::PublisherBase::SharedPtr joint_state_pub_;
};

// Segment-wise kinematics helpers (free functions).
// These operate on CLegSegmentwise and CPose/CPosition and are implemented in
// src/requester/kinematics.cpp. They are intentionally free functions so tests
// and utility code can call them without constructing a CKinematics instance.
CPosition computeFootPositionSegmentwise(const CPosition& hip_base, const CLegSegmentwise& leg);
CPosition computeFootPositionSegmentwise(const CPose& hip_pose, const CLegSegmentwise& leg);
bool solveIKSegmentwise(const CPose& hip_pose, const CPosition& foot_pos, CLegSegmentwise& leg,
                        double tolerance = 1e-6);
