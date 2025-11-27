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
    EXPECT_NEAR(expected.coxa_deg, actual.coxa_deg, tolerance) << msg;
    EXPECT_NEAR(expected.femur_deg, actual.femur_deg, tolerance) << msg;
    EXPECT_NEAR(expected.tibia_deg, actual.tibia_deg, tolerance) << msg;
}

// Compare CPose component-wise (position + orientation)
inline void expectPoseNear(const CPose& expected, const CPose& actual, const std::string& msg = "") {
    expectPositionNear(expected.position, actual.position, msg);
    EXPECT_DOUBLE_EQ(expected.orientation.roll_deg, actual.orientation.roll_deg) << msg;
    EXPECT_DOUBLE_EQ(expected.orientation.pitch_deg, actual.orientation.pitch_deg) << msg;
    EXPECT_DOUBLE_EQ(expected.orientation.yaw_deg, actual.orientation.yaw_deg) << msg;
}

// Compare CLeg (angles + foot position)
inline void expectLegNear(const CLeg& expected, const CLeg& actual, const std::string& msg = "") {
    expectAnglesNear(expected.angles_deg_, actual.angles_deg_, msg);
    expectPositionNear(expected.foot_pos_, actual.foot_pos_, msg);
}

// Compare CHead (yaw, pitch)
inline void expectHeadNear(const CHead& expected, const CHead& actual, const std::string& msg = "") {
    EXPECT_DOUBLE_EQ(expected.yaw_deg, actual.yaw_deg) << msg;
    EXPECT_DOUBLE_EQ(expected.pitch_deg, actual.pitch_deg) << msg;
}
