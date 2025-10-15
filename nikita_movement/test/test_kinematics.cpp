#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"

using namespace std;

std::shared_ptr<rclcpp::Node> makeNode() {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    return std::make_shared<rclcpp::Node>("test_kinematics_node");
}

TEST(KinematicsForward, BasicGeometry) {
    auto node = makeNode();
    CKinematics kin(node);

    // set minimal parameters required by CKinematics constructor and usage
    kin.LEG_NAMES = {"RightFront"};
    kin.COXA_LENGTH = 0.050;
    kin.FEMUR_LENGTH = 0.063;
    kin.TIBIA_LENGTH = 0.099;
    kin.COXA_HEIGHT = 0.045;

    // body center offsets for the single leg
    kin.CENTER_TO_COXA_X = {0.109};
    kin.CENTER_TO_COXA_Y = {0.068};
    kin.OFFSET_COXA_ANGLE_DEG = {45.0};

    // create target angles
    CLegAngles angles;
    angles.degCoxa = 0.0;
    angles.degFemur = 2.3;
    angles.degTibia = 7.7;

    kin.setLegAngles(ELegIndex::RightFront, angles);

    double expected_x = 0.092;
    double expected_y = 0.092;
    double expected_z = -0.050;

    auto& leg = kin.getLegs().at(ELegIndex::RightFront);

    EXPECT_NEAR(leg.footPos_.x, expected_x, 1e-6);
    EXPECT_NEAR(leg.footPos_.y, expected_y, 1e-6);
    EXPECT_NEAR(leg.footPos_.z, expected_z, 1e-6);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
