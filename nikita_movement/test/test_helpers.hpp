#pragma once

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "requester/gaits_parameter.hpp"
#include "requester/kinematics.hpp"

namespace nikita_movement::test_helpers {

inline std::vector<rclcpp::Parameter> defaultKinematicsParameters() {
    return {
        rclcpp::Parameter("LEG_NAMES", std::vector<std::string>{"RightFront", "RightMid", "RightBack",
                                                                "LeftFront", "LeftMid", "LeftBack"}),
        rclcpp::Parameter("COXA_LENGTH", 0.050),
        rclcpp::Parameter("FEMUR_LENGTH", 0.063),
        rclcpp::Parameter("TIBIA_LENGTH", 0.099),
        rclcpp::Parameter("COXA_HEIGHT", 0.045),
        rclcpp::Parameter("CENTER_TO_COXA_X", std::vector<double>{0.109, 0.0, -0.109, 0.109, 0.0, -0.109}),
        rclcpp::Parameter("CENTER_TO_COXA_Y",
                          std::vector<double>{0.068, 0.088, 0.068, -0.068, -0.088, -0.068}),
        rclcpp::Parameter("OFFSET_COXA_ANGLE_DEG",
                          std::vector<double>{45.0, 90.0, 135.0, -45.0, -90.0, -135.0})};
}

inline std::vector<rclcpp::Parameter> defaultGaitParameters() {
    return {rclcpp::Parameter("LEG_LIFT_HEIGHT", 0.03),
            rclcpp::Parameter("BODY_MAX_ROLL", 20.0),
            rclcpp::Parameter("BODY_MAX_PITCH", 10.0),
            rclcpp::Parameter("HEAD_MAX_YAW_TRIPOD", 15.0),
            rclcpp::Parameter("FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME", 0.5),
            rclcpp::Parameter("GAIT_STEP_LENGTH", 0.05),
            rclcpp::Parameter("GAIT_LEG_WAVE_LEG_LIFT_HEIGHT", 0.025),
            rclcpp::Parameter("GAIT_LOOK_BODY_MAX_YAW", 10.0),
            rclcpp::Parameter("GAIT_LOOK_HEAD_MAX_YAW", 12.0),
            rclcpp::Parameter("GAIT_WATCH_BODY_MAX_YAW", 8.0),
            rclcpp::Parameter("GAIT_WATCH_HEAD_MAX_YAW", 15.0),
            rclcpp::Parameter("TESTLEGS_COXA_DELTA_DEG", 5.0),
            rclcpp::Parameter("TESTLEGS_FEMUR_DELTA_DEG", 5.0),
            rclcpp::Parameter("TESTLEGS_TIBIA_DELTA_DEG", 5.0),
            rclcpp::Parameter("TESTLEGS_HOLD_TIME_PER_LEG", 0.5)};
}

inline std::vector<rclcpp::Parameter> defaultRobotParameters() {
    auto params = defaultKinematicsParameters();
    const auto gait_params = defaultGaitParameters();
    params.insert(params.end(), gait_params.begin(), gait_params.end());
    return params;
}

inline Parameters makeDeclaredParameters(const std::shared_ptr<rclcpp::Node>& node) {
    return Parameters::declare(node);
}

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

}  // namespace nikita_movement::test_helpers

using nikita_movement::test_helpers::expectAnglesNear;
using nikita_movement::test_helpers::expectHeadNear;
using nikita_movement::test_helpers::expectLegNear;
using nikita_movement::test_helpers::expectPoseNear;
using nikita_movement::test_helpers::expectPositionNear;
