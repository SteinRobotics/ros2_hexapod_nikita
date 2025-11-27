/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <magic_enum.hpp>
#include <map>
#include <memory>
#include <string>

#include "nikita_interfaces/msg/pose.hpp"

enum class ELegIndex {
    RightFront,
    RightMid,
    RightBack,
    LeftFront,
    LeftMid,
    LeftBack,
};

inline ELegIndex legNameToIndex(std::string_view name) {
    return magic_enum::enum_cast<ELegIndex>(name).value();
}

inline std::string legIndexToName(ELegIndex index) {
    return std::string(magic_enum::enum_name(index));
}

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

    bool operator==(const CPosition& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
    bool operator!=(const CPosition& rhs) const {
        return !(*this == rhs);
    }

    static inline bool almostEqual(const CPosition& a, const CPosition& b, double tol) {
        return (std::abs(a.x - b.x) <= tol) && (std::abs(a.y - b.y) <= tol) && (std::abs(a.z - b.z) <= tol);
    }

    double x = double(0);
    double y = double(0);
    double z = double(0);

    // Linear interpolation member: returns a point between this and 'target' at parameter alpha in [0,1]
    inline CPosition linearInterpolate(const CPosition& target, double alpha) const {
        return CPosition(x + (target.x - x) * alpha, y + (target.y - y) * alpha, z + (target.z - z) * alpha);
    }
};

class COrientation {
   public:
    COrientation() = default;
    COrientation(double roll, double pitch, double yaw) : roll(roll), pitch(pitch), yaw(yaw) {};
    virtual ~COrientation() = default;

    double roll = double(0);
    double pitch = double(0);
    double yaw = double(0);

    // Linear interpolation member: returns an orientation between this and 'target' at parameter alpha in [0,1]
    inline COrientation linearInterpolate(const COrientation& target, double alpha) const {
        return COrientation(roll + (target.roll - roll) * alpha, pitch + (target.pitch - pitch) * alpha,
                            yaw + (target.yaw - yaw) * alpha);
    }
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

    // Linear interpolation member: interpolate position and orientation
    inline CPose linearInterpolate(const CPose& target, double alpha) const {
        CPose out;
        out.position = position.linearInterpolate(target.position, alpha);
        out.orientation = orientation.linearInterpolate(target.orientation, alpha);
        return out;
    }
};

class CBodyCenterOffset {
   public:
    double x = double(0);
    double y = double(0);
    double psi = double(0);
};

class CLegAngles {
   public:
    CLegAngles(double coxa_deg, double femur_deg, double tibia_deg)
        : coxa_deg(coxa_deg), femur_deg(femur_deg), tibia_deg(tibia_deg) {};
    CLegAngles() = default;
    virtual ~CLegAngles() = default;

    double coxa_deg = double(0);
    double femur_deg = double(0);
    double tibia_deg = double(0);

    // Linear interpolation member: interpolate each joint angle (degrees)
    inline CLegAngles linearInterpolate(const CLegAngles& target, double alpha) const {
        return CLegAngles(coxa_deg + (target.coxa_deg - coxa_deg) * alpha,
                          femur_deg + (target.femur_deg - femur_deg) * alpha,
                          tibia_deg + (target.tibia_deg - tibia_deg) * alpha);
    }
};

class CLeg {
   public:
    CLeg() = default;
    CLeg(CLegAngles angles_deg, CPosition foot_pos) : angles_deg_(angles_deg), foot_pos_(foot_pos) {};

    CLegAngles angles_deg_;
    CPosition foot_pos_;
};

class CHead {
   public:
    CHead() = default;
    CHead(double yaw_deg, double pitch_deg) : yaw_deg(yaw_deg), pitch_deg(pitch_deg) {};
    virtual ~CHead() = default;

    double yaw_deg = double(0);
    double pitch_deg = double(0);
};

// --------------------------------------------------------
// ------------------  for future usage ------------------
struct CJointDesc {
    double offset_rad = 0.0;
    double limit_min_rad = -M_PI_2;
    double limit_max_rad = M_PI_2;
    // axis info if needed later
};

struct CJointState {
    double angle_rad = 0.0;
    bool dirty = true;
};

struct CLink {
    double length_m = 0.0;
};

struct CSegment {
    CLink link;         // geometry: length
    CJointDesc desc;    // geometry/limit/offset
    CJointState state;  // runtime
};

struct CLegSegmentwise {
    CSegment coxa;
    CSegment femur;
    CSegment tibia;
};

struct CBody {
    CPose pose;
    std::map<ELegIndex, CLegSegmentwise> legs;
    std::map<ELegIndex, CBodyCenterOffset> bodyCenterOffsets;
};
