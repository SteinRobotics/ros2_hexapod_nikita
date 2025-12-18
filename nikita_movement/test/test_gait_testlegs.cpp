#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "requester/gait_testlegs.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace nikita_movement;

namespace {
constexpr double kTolerance = 1e-4;
constexpr int kMaxIterations = 200;
}  // namespace

class TestLegsGaitTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        overrides.emplace_back("TESTLEGS_STAGE_DURATION", 0.0);
        overrides.emplace_back("TESTLEGS_MIN_STAGE_DURATION", 0.0);
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_testlegs_node", options);
        actionPackagesParser_ = std::make_shared<CActionPackagesParser>(node_);
        kinematics_ = std::make_shared<CKinematics>(node_, actionPackagesParser_);
        params_ = test_helpers::makeDeclaredParameters(node_);
    }

    void TearDown() override {
        kinematics_.reset();
        actionPackagesParser_.reset();
        node_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPackagesParser> actionPackagesParser_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters params_;
};

TEST_F(TestLegsGaitTest, RaisesAtLeastOneLegAndReturnsToBase) {
    CTestLegsGait gait(node_, kinematics_, params_.testLegs);
    std::map<ELegIndex, CLegAngles> base_angles;
    for (const auto& [index, leg] : kinematics_->getLegs()) {
        base_angles[index] = leg.angles_deg_;
    }

    gait.start(0.0, 0);
    geometry_msgs::msg::Twist twist;
    bool observed_raise = false;

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        EXPECT_TRUE(gait.update(twist, CPose()));
        for (const auto& [index, leg] : kinematics_->getLegs()) {
            const auto& base = base_angles.at(index);
            if (leg.angles_deg_.coxa_deg >= base.coxa_deg + 4.0 &&
                leg.angles_deg_.femur_deg >= base.femur_deg + 4.0 &&
                leg.angles_deg_.tibia_deg >= base.tibia_deg + 4.0) {
                observed_raise = true;
                break;
            }
        }
    }

    EXPECT_TRUE(observed_raise);

    for (const auto& [index, leg] : kinematics_->getLegs()) {
        const auto& base = base_angles.at(index);
        EXPECT_NEAR(leg.angles_deg_.coxa_deg, base.coxa_deg, kTolerance);
        EXPECT_NEAR(leg.angles_deg_.femur_deg, base.femur_deg, kTolerance);
        EXPECT_NEAR(leg.angles_deg_.tibia_deg, base.tibia_deg, kTolerance);
    }
}

TEST_F(TestLegsGaitTest, RequestStopRestoresImmediately) {
    CTestLegsGait gait(node_, kinematics_, params_.testLegs);
    std::map<ELegIndex, CLegAngles> base_angles;
    for (const auto& [index, leg] : kinematics_->getLegs()) {
        base_angles[index] = leg.angles_deg_;
    }

    gait.start(0.0, 0);
    geometry_msgs::msg::Twist twist;

    gait.update(twist, CPose());  // raise first leg
    gait.requestStop();

    EXPECT_FALSE(gait.update(twist, CPose()));
    EXPECT_EQ(gait.state(), EGaitState::Stopped);

    for (const auto& [index, leg] : kinematics_->getLegs()) {
        const auto& base = base_angles.at(index);
        EXPECT_NEAR(leg.angles_deg_.coxa_deg, base.coxa_deg, kTolerance);
        EXPECT_NEAR(leg.angles_deg_.femur_deg, base.femur_deg, kTolerance);
        EXPECT_NEAR(leg.angles_deg_.tibia_deg, base.tibia_deg, kTolerance);
    }
}
