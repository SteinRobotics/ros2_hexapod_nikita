#pragma once

#include <gtest/gtest.h>

#include "requester/kinematics.hpp"

// Reusable helper to compare CPosition
inline void expectPositionNear(const CPosition& expected, const CPosition& actual,
                               const std::string& msg = "", double tolerance = 1e-3) {
    EXPECT_NEAR(expected.x, actual.x, tolerance) << msg;
    EXPECT_NEAR(expected.y, actual.y, tolerance) << msg;
    EXPECT_NEAR(expected.z, actual.z, tolerance) << msg;
}

// Reusable helper to compare CLegAngles
inline void expectAnglesNear(const CLegAngles& expected, const CLegAngles& actual,
                             const std::string& msg = "", double tolerance = 1e-3) {
    EXPECT_NEAR(expected.degCoxa, actual.degCoxa, tolerance) << msg;
    EXPECT_NEAR(expected.degFemur, actual.degFemur, tolerance) << msg;
    EXPECT_NEAR(expected.degTibia, actual.degTibia, tolerance) << msg;
}

// Compare CPose component-wise (position + orientation)
inline void expectPoseNear(const CPose& expected, const CPose& actual, const std::string& msg = "") {
    expectPositionNear(expected.position, actual.position, msg);
    EXPECT_DOUBLE_EQ(expected.orientation.roll, actual.orientation.roll) << msg;
    EXPECT_DOUBLE_EQ(expected.orientation.pitch, actual.orientation.pitch) << msg;
    EXPECT_DOUBLE_EQ(expected.orientation.yaw, actual.orientation.yaw) << msg;
}

// Compare CLeg (angles + foot position)
inline void expectLegNear(const CLeg& expected, const CLeg& actual, const std::string& msg = "") {
    expectAnglesNear(expected.angles_, actual.angles_, msg);
    expectPositionNear(expected.footPos_, actual.footPos_, msg);
}

// Compare CHead (yaw, pitch)
inline void expectHeadNear(const CHead& expected, const CHead& actual, const std::string& msg = "") {
    EXPECT_DOUBLE_EQ(expected.degYaw, actual.degYaw) << msg;
    EXPECT_DOUBLE_EQ(expected.degPitch, actual.degPitch) << msg;
}
