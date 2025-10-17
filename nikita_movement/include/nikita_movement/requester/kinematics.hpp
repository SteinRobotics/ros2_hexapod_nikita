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
#include "rclcpp/rclcpp.hpp"

// future thoughts:
// CBody has 1 CPose
// CBody has 6 CLegs and 1 CHead
// CLeg has 3 CJoints
// CHead has 2 CJoints
// CJoint has 1 degAngle, has 1 CPosition (x,y,z), has 1 index(Name)

// or
// CLeg has 3 Joints, each Joint has 1 parentJoint and 1 childJoint

enum class ELegIndex {
    RightFront,
    RightMid,
    RightBack,
    LeftFront,
    LeftMid,
    LeftBack,
};

// TODO is there better way than using global variables?
const std::map<const ELegIndex, const std::string> legIndexToName = {
    {ELegIndex::RightFront, "RightFront"}, {ELegIndex::RightMid, "RightMid"},
    {ELegIndex::RightBack, "RightBack"},   {ELegIndex::LeftFront, "LeftFront"},
    {ELegIndex::LeftMid, "LeftMid"},       {ELegIndex::LeftBack, "LeftBack"},
};
const std::map<const std::string, const ELegIndex> legNameToIndex = {
    {"RightFront", ELegIndex::RightFront}, {"RightMid", ELegIndex::RightMid},
    {"RightBack", ELegIndex::RightBack},   {"LeftFront", ELegIndex::LeftFront},
    {"LeftMid", ELegIndex::LeftMid},       {"LeftBack", ELegIndex::LeftBack},
};

class CPosition {
   public:
    CPosition() = default;
    CPosition(double x, double y, double z) : x(x), y(y), z(z) {};
    virtual ~CPosition() = default;
    CPosition operator+(const CPosition& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }
    CPosition operator-(const CPosition& rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    bool operator==(const CPosition &rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    bool operator!=(const CPosition &rhs) const { return !(*this == rhs); }

    // approximate comparison with tolerance
    static inline bool almostEqual(const CPosition &a, const CPosition &b, double tol) {
        return (std::abs(a.x - b.x) <= tol) && (std::abs(a.y - b.y) <= tol) && (std::abs(a.z - b.z) <= tol);
    }

    double x = double(0);
    double y = double(0);
    double z = double(0);
};

class COrientation {
   public:
    COrientation() = default;
    COrientation(double roll, double pitch, double yaw) : roll(roll), pitch(pitch), yaw(yaw) {};
    virtual ~COrientation() = default;

    double roll = double(0);
    double pitch = double(0);
    double yaw = double(0);
};

class CPose {
   public:
    CPose() = default;
    CPose(double x, double y, double z, double roll, double pitch, double yaw)
        : position(x, y, z), orientation(roll, pitch, yaw) {};
    CPose(CPosition position, COrientation orientation) : position(position), orientation(orientation) {};

    CPose(const nikita_interfaces::msg::Pose& pose)
        : position(pose.position.x, pose.position.y, pose.position.z),
          orientation(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw) {};

    virtual ~CPose() = default;

    CPosition position;
    COrientation orientation;
};

class CBodyCenterOffset {
   public:
    double x = double(0);
    double y = double(0);
    double psi = double(0);
};

class CLegAngles {
   public:
    CLegAngles(double degCoxa, double degFemur, double degTibia)
        : degCoxa(degCoxa), degFemur(degFemur), degTibia(degTibia) {};
    CLegAngles() = default;
    virtual ~CLegAngles() = default;

    double degCoxa = double(0);
    double degFemur = double(0);
    double degTibia = double(0);
};

class CLeg {
   public:
    CLeg() = default;
    CLeg(CLegAngles angles, CPosition footPos) : angles_(angles), footPos_(footPos) {};

    CLegAngles angles_;
    CPosition footPos_;
};

class CHead {
   public:
    CHead() = default;
    CHead(double degYaw, double degPitch) : degYaw(degYaw), degPitch(degPitch) {};
    virtual ~CHead() = default;

    double degYaw = double(0);
    double degPitch = double(0);
};

class CKinematics {
   public:
    CKinematics(std::shared_ptr<rclcpp::Node> node);
    ~CKinematics() = default;

    void setSingleFeet(const ELegIndex index, const CPosition& targetFeetPos);
    void setLegAngles(const ELegIndex index, const CLegAngles& angles);
    void moveBody(const std::map<ELegIndex, CPosition>& footTargets,
                  const CPose body = CPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    void setHead(double degYaw, double degPitch) {
        head_.degYaw = degYaw;
        head_.degPitch = degPitch;
    };
    // set head by components
    void setHead(CHead head) {
        head_.degYaw = head.degYaw;
        head_.degPitch = head.degPitch;
    }

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

    // Parameters
    std::vector<std::string> LEG_NAMES;

    double COXA_LENGTH = double(0);
    double COXA_HEIGHT = double(0);
    double FEMUR_LENGTH = double(0);
    double TIBIA_LENGTH = double(0);

    std::vector<double> CENTER_TO_COXA_X;
    std::vector<double> CENTER_TO_COXA_Y;
    std::vector<double> OFFSET_COXA_ANGLE_DEG;

    std::vector<double> STANDING_FOOT_POS_X;
    std::vector<double> STANDING_FOOT_POS_Y;
    std::vector<double> STANDING_FOOT_POS_Z;

    std::vector<double> LAYDOWN_FOOT_POS_X;
    std::vector<double> LAYDOWN_FOOT_POS_Y;
    std::vector<double> LAYDOWN_FOOT_POS_Z;

    double BODY_MAX_ROLL = double(0);
    double BODY_MAX_PITCH = double(0);
    double BODY_MAX_YAW = double(0);
    double HEAD_MAX_YAW = double(0);
    double HEAD_MAX_PITCH = double(0);

   private:
    void intializeLegs(std::map<ELegIndex, CLeg>& legs, std::vector<double>& posX, std::vector<double>& posY,
                       std::vector<double>& posZ);
    void logLegsPositions(std::map<ELegIndex, CLeg>& legs);
    void logLegPosition(const ELegIndex index, const CLeg& leg);
    void calcLegInverseKinematics(const CPosition& targetFeetPos, CLeg& leg, const ELegIndex& legIndex);
    void calcLegForwardKinematics(const CLegAngles target, CLeg& leg);
    CPosition rotate(const CPosition& point, const COrientation& rot);

    std::shared_ptr<rclcpp::Node> node_;

    std::map<ELegIndex, CLeg> legs_;          // current values
    std::map<ELegIndex, CLeg> legsStanding_;  // change to shared pointer and make const
    std::map<ELegIndex, CLeg> legsLayDown_;   // change to shared pointer and make const
    std::map<ELegIndex, CBodyCenterOffset> bodyCenterOffsets_;

    CPose body_;
    CHead head_;

    double sqFemurLength_ = 0.0;
    double sqTibiaLength_ = 0.0;
};
