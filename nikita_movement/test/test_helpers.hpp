#pragma once

#include <gtest/gtest.h>
#include "requester/kinematics.hpp"

// Reusable helper to compare CPosition with a tolerance
inline void expectPositionNear(const CPosition &expected, const CPosition &actual, double tol,
                               const std::string &msg = "") {
    EXPECT_NEAR(expected.x, actual.x, tol) << msg;
    EXPECT_NEAR(expected.y, actual.y, tol) << msg;
    EXPECT_NEAR(expected.z, actual.z, tol) << msg;
}

// Reusable helper to compare CLegAngles with a tolerance
inline void expectAnglesNear(const CLegAngles &expected, const CLegAngles &actual, double tol,
                             const std::string &msg = "") {
    EXPECT_NEAR(expected.degCoxa, actual.degCoxa, tol) << msg;
    EXPECT_NEAR(expected.degFemur, actual.degFemur, tol) << msg;
    EXPECT_NEAR(expected.degTibia, actual.degTibia, tol) << msg;
}

// Compare CPose component-wise (position + orientation)
inline void expectPoseNear(const CPose &expected, const CPose &actual, double tol,
                          const std::string &msg = "") {
    expectPositionNear(expected.position, actual.position, tol, msg);
    EXPECT_NEAR(expected.orientation.roll, actual.orientation.roll, tol) << msg;
    EXPECT_NEAR(expected.orientation.pitch, actual.orientation.pitch, tol) << msg;
    EXPECT_NEAR(expected.orientation.yaw, actual.orientation.yaw, tol) << msg;
}

// Compare CLeg (angles + foot position)
inline void expectLegNear(const CLeg &expected, const CLeg &actual, double tol,
                        const std::string &msg = "") {
    expectAnglesNear(expected.angles_, actual.angles_, tol, msg);
    expectPositionNear(expected.footPos_, actual.footPos_, tol, msg);
}
