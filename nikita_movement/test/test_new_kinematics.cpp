#include <gtest/gtest.h>

#include "nikita_utils/geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"
#include "test_helpers.hpp"

using namespace std;
using namespace utils;

class NewKinematicsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("LEG_NAMES", std::vector<std::string>{"RightFront", "RightMid", "RightBack",
                                                                    "LeftFront", "LeftMid", "LeftBack"}),
            rclcpp::Parameter("COXA_LENGTH", 0.050),
            rclcpp::Parameter("FEMUR_LENGTH", 0.063),
            rclcpp::Parameter("TIBIA_LENGTH", 0.099),
            rclcpp::Parameter("COXA_HEIGHT", 0.045),
            rclcpp::Parameter("CENTER_TO_COXA_X",
                              std::vector<double>{0.109, 0.0, -0.109, 0.109, 0.0, -0.109}),
            rclcpp::Parameter("CENTER_TO_COXA_Y",
                              std::vector<double>{0.068, 0.088, 0.068, -0.068, -0.088, -0.068}),
            rclcpp::Parameter("OFFSET_COXA_ANGLE_DEG",
                              std::vector<double>{45.0, 90.0, 135.0, -45.0, -90.0, -135.0}),
        });
        node_ = std::make_shared<rclcpp::Node>("test_new_kinematics_node", options);
        actionPackagesParser_ = std::make_shared<CActionPackagesParser>(node_);
        kin_ = std::make_unique<CKinematics>(node_, actionPackagesParser_);
    }

    void TearDown() override {
        kin_.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPackagesParser> actionPackagesParser_;
    std::unique_ptr<CKinematics> kin_;
};

TEST_F(NewKinematicsTest, CKinematics_IK_FK_roundtrip) {
    // pick a representative leg and a standing target
    const ELegIndex idx = ELegIndex::RightFront;
    auto footTargets = kin_->getLegsStandingPositions();
    auto target = footTargets.at(idx);

    // solve IK using CKinematics (via setSingleFeet) and then recompute FK using setLegAngles
    kin_->setSingleFeet(idx, target);
    auto angles = kin_->getLegs().at(idx).angles_deg_;

    // recompute forward kinematics
    kin_->setLegAngles(idx, angles);
    auto recomposed = kin_->getLegs().at(idx).foot_pos_;

    double err = std::sqrt(std::pow(recomposed.x - target.x, 2) + std::pow(recomposed.y - target.y, 2) +
                           std::pow(recomposed.z - target.z, 2));

    EXPECT_LT(err, 1e-3) << "CKinematics IK->FK roundtrip error too large";
}

TEST_F(NewKinematicsTest, Segmentwise_IK_FK_roundtrip) {
    // build a segmentwise leg and test FK->IK->FK consistency
    const ELegIndex idx = ELegIndex::RightFront;

    CLegSegmentwise leg;
    leg.coxa.link.length_m = kin_->COXA_LENGTH;
    leg.femur.link.length_m = kin_->FEMUR_LENGTH;
    leg.tibia.link.length_m = kin_->TIBIA_LENGTH;

    // wide joint limits for the test
    leg.coxa.desc.limit_min_rad = -M_PI;
    leg.coxa.desc.limit_max_rad = M_PI;
    leg.femur.desc.limit_min_rad = -M_PI;
    leg.femur.desc.limit_max_rad = M_PI;
    leg.tibia.desc.limit_min_rad = -M_PI;
    leg.tibia.desc.limit_max_rad = M_PI;

    // choose a plausible pose (in radians)
    leg.coxa.state.angle_rad = deg2rad(0.0);
    leg.femur.state.angle_rad = deg2rad(20.0);
    leg.tibia.state.angle_rad = deg2rad(-10.0);

    // hip pose (use the same offsets as CKinematics initialize)
    CPose hip_pose;
    hip_pose.position.x = kin_->CENTER_TO_COXA_X.at(static_cast<size_t>(idx));
    hip_pose.position.y = kin_->CENTER_TO_COXA_Y.at(static_cast<size_t>(idx));
    hip_pose.position.z = kin_->COXA_HEIGHT;  // hip height
    hip_pose.orientation.roll_deg = hip_pose.orientation.pitch_deg = hip_pose.orientation.yaw_deg = 0.0;

    // forward FK (segmentwise)
    CPosition foot = computeFootPositionSegmentwise(hip_pose, leg);

    // try to recover the angles with the IK solver
    CLegSegmentwise leg_recovered = leg;
    // zero the state to simulate an empty initial guess
    leg_recovered.coxa.state.angle_rad = 0.0;
    leg_recovered.femur.state.angle_rad = 0.0;
    leg_recovered.tibia.state.angle_rad = 0.0;

    bool ok = solveIKSegmentwise(hip_pose, foot, leg_recovered);
    EXPECT_TRUE(ok) << "Segmentwise IK reported unreachable target";

    // recompute FK from recovered angles and compare
    CPosition foot2 = computeFootPositionSegmentwise(hip_pose, leg_recovered);
    expectPositionNear(foot, foot2, "segmentwise FK/IK mismatch", 1e-6);
}

TEST_F(NewKinematicsTest, Compare_old_vs_new_errors) {
    // Compare numerical recomposition errors from CKinematics and segmentwise approach
    const ELegIndex idx = ELegIndex::RightFront;

    // CKinematics error
    auto target = kin_->getLegsStandingPositions().at(idx);
    kin_->setSingleFeet(idx, target);
    auto angles = kin_->getLegs().at(idx).angles_deg_;
    kin_->setLegAngles(idx, angles);
    auto recomposed = kin_->getLegs().at(idx).foot_pos_;
    double err_ck = std::sqrt(std::pow(recomposed.x - target.x, 2) + std::pow(recomposed.y - target.y, 2) +
                              std::pow(recomposed.z - target.z, 2));

    // Segmentwise error (use the same joint angles transformed to radians)
    CLegSegmentwise leg;
    leg.coxa.link.length_m = kin_->COXA_LENGTH;
    leg.femur.link.length_m = kin_->FEMUR_LENGTH;
    leg.tibia.link.length_m = kin_->TIBIA_LENGTH;
    leg.coxa.state.angle_rad =
        deg2rad(angles.coxa_deg - kin_->OFFSET_COXA_ANGLE_DEG.at(static_cast<size_t>(idx)));
    leg.femur.state.angle_rad = deg2rad(angles.femur_deg);
    leg.tibia.state.angle_rad = deg2rad(angles.tibia_deg);
    CPose hip_pose;
    hip_pose.position.x = kin_->CENTER_TO_COXA_X.at(static_cast<size_t>(idx));
    hip_pose.position.y = kin_->CENTER_TO_COXA_Y.at(static_cast<size_t>(idx));
    hip_pose.position.z = kin_->COXA_HEIGHT;

    CPosition foot_seg = computeFootPositionSegmentwise(hip_pose, leg);
    // Attempt IK recover and recompute
    CLegSegmentwise leg_rec = leg;
    leg_rec.coxa.state.angle_rad = 0.0;
    leg_rec.femur.state.angle_rad = 0.0;
    leg_rec.tibia.state.angle_rad = 0.0;
    bool ok = solveIKSegmentwise(hip_pose, foot_seg, leg_rec);
    EXPECT_TRUE(ok) << "segmentwise IK unreachable for standing pose";
    CPosition foot_seg_rec = computeFootPositionSegmentwise(hip_pose, leg_rec);
    double err_seg =
        std::sqrt(std::pow(foot_seg_rec.x - foot_seg.x, 2) + std::pow(foot_seg_rec.y - foot_seg.y, 2) +
                  std::pow(foot_seg_rec.z - foot_seg.z, 2));

    // expect both methods to be reasonably accurate
    EXPECT_LT(err_ck, 1e-3) << "CKinematics recomposition error too large";
    EXPECT_LT(err_seg, 1e-6) << "Segmentwise recomposition error too large";
}
