#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace std;

class KinematicsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("LEG_NAMES", std::vector<std::string>{"RightFront"}),
            rclcpp::Parameter("COXA_LENGTH", 0.050),
            rclcpp::Parameter("FEMUR_LENGTH", 0.063),
            rclcpp::Parameter("TIBIA_LENGTH", 0.099),
            rclcpp::Parameter("COXA_HEIGHT", 0.045),
            rclcpp::Parameter("CENTER_TO_COXA_X", std::vector<double>{0.109}),
            rclcpp::Parameter("CENTER_TO_COXA_Y", std::vector<double>{0.068}),
            rclcpp::Parameter("OFFSET_COXA_ANGLE_DEG", std::vector<double>{45.0}),
            // standing and laydown positions for the single leg
            rclcpp::Parameter("STANDING_FOOT_POS_X", std::vector<double>{0.092}),
            rclcpp::Parameter("STANDING_FOOT_POS_Y", std::vector<double>{0.092}),
            rclcpp::Parameter("STANDING_FOOT_POS_Z", std::vector<double>{-0.050}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_X", std::vector<double>{0.071}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_Y", std::vector<double>{0.071}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_Z", std::vector<double>{0.010}),
        });

        node_ = std::make_shared<rclcpp::Node>("test_kinematics_node", options);
        kin_ = std::make_unique<CKinematics>(node_);
        cout << "KinematicsTest SetUp complete" << endl;
    }

    void TearDown() override {
        kin_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CKinematics> kin_;
};

// Standing position
// RightFront: 	ag: 0.000°, 2.276°, 7.704°	| x: 0.201, y: 0.160, z: -0.050

// Laydown position
// RightFront: 	ag: 0.000°, 70.723°, -53.320°	| x: 0.180, y: 0.139, z: 0.010


TEST_F(KinematicsTest, setLegAngles) {
    // create target angles
    CLegAngles angles;
    angles.degCoxa = 0.0;
    angles.degFemur = 2.276;
    angles.degTibia =  7.704;

    kin_->setLegAngles(ELegIndex::RightFront, angles);

    CPosition footPosExpected;
    footPosExpected.x = 0.201; // 0.092 + 0.109
    footPosExpected.y = 0.160; // 0.092 + 0.068
    footPosExpected.z = -0.050; // 0.045 - 0.095

    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    expectPositionNear(footPosExpected, leg.footPos_, 1e-3, "setLegAngles position mismatch");
    expectAnglesNear(angles, leg.angles_, 1e-3, "setLegAngles angles mismatch");
}

TEST_F(KinematicsTest, checkStandingPosition) {
    kin_->moveBody(kin_->getLegsStandingPositions());
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CPosition footPosExpected;
    footPosExpected.x = 0.201; // 0.092 + 0.109
    footPosExpected.y = 0.160; // 0.092 + 0.068
    footPosExpected.z = -0.050; // 0.045 - 0.095

    CLegAngles anglesExpected;
    anglesExpected.degCoxa = 0.0;
    anglesExpected.degFemur = 2.276;
    anglesExpected.degTibia = 7.704;

    expectPositionNear(footPosExpected, leg.footPos_, 1e-3, "standing position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_, 1e-3, "standing angles mismatch");
}

TEST_F(KinematicsTest, checkLaydownPosition) {
    kin_->moveBody(kin_->getLegsLayDownPositions());
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CPosition footPosExpected;
    footPosExpected.x = 0.180; // 0.071 + 0.109
    footPosExpected.y = 0.139; // 0.071 + 0.068
    footPosExpected.z = 0.010;  // 0.045 - 0.035

    CLegAngles anglesExpected;
    anglesExpected.degCoxa = 0.0;
    anglesExpected.degFemur = 70.723;
    anglesExpected.degTibia = -53.320;

    expectPositionNear(footPosExpected, leg.footPos_, 1e-3, "laydown position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_, 1e-3, "laydown angles mismatch");
}

TEST_F(KinematicsTest, checkSetFeet) {
    CPosition targetPos;
    targetPos.x = 0.201; // 0.092 + 0.109
    targetPos.y = 0.160; // 0.092 + 0.068
    targetPos.z = -0.050; // 0.045 - 0.095

    kin_->setSingleFeet(ELegIndex::RightFront, targetPos);
    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    CLegAngles anglesExpected;
    anglesExpected.degCoxa = 0.0;
    anglesExpected.degFemur = 2.276;
    anglesExpected.degTibia = 7.704;

    expectPositionNear(targetPos, leg.footPos_, 1e-3, "setFeet position mismatch");
    expectAnglesNear(anglesExpected, leg.angles_, 1e-3, "setFeet angles mismatch");
}