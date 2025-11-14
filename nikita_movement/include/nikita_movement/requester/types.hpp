/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

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
