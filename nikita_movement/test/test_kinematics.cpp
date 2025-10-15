#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"

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
// RightFront: 	ag:   0.0°,   2.3°,   7.7°	| x: 0.20, y: 0.16, z: -0.05

// Laydown position
// RightFront: 	ag:   0.0°,  71°, -53°	| x: 0.18, y: 0.14, z: 0.01

TEST_F(KinematicsTest, setLegAngles) {
    // create target angles
    CLegAngles angles;
    angles.degCoxa = 0.0;
    angles.degFemur = 2.3;
    angles.degTibia = 7.7;

    kin_->setLegAngles(ELegIndex::RightFront, angles);

    double expected_x = 0.109;
    double expected_y = 0.1981404164812362;
    double expected_z = -0.050;

    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    EXPECT_NEAR(leg.footPos_.x, expected_x, 1e-4);
    EXPECT_NEAR(leg.footPos_.y, expected_y, 1e-4);
    EXPECT_NEAR(leg.footPos_.z, expected_z, 1e-4);
}

TEST_F(KinematicsTest, moveBody) {
    std::map<ELegIndex, CPosition> footTargets;
    footTargets[ELegIndex::RightFront] = CPosition(0.109, 0.1981404164812362, -0.050);

    CPose body;
    body.position.x = 0.0;
    body.position.y = 0.0;
    body.position.z = 0.0;
    body.orientation.roll = 0.0;
    body.orientation.pitch = 0.0;
    body.orientation.yaw = 0.0;

    kin_->moveBody(footTargets, body);

    double expected_x = 0.109;
    double expected_y = 0.1981404164812362;
    double expected_z = -0.050;

    auto& leg = kin_->getLegs().at(ELegIndex::RightFront);

    EXPECT_NEAR(leg.footPos_.x, expected_x, 1e-4);
    EXPECT_NEAR(leg.footPos_.y, expected_y, 1e-4);
    EXPECT_NEAR(leg.footPos_.z, expected_z, 1e-4);
}