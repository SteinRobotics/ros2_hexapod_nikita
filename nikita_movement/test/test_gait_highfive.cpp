#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "requester/gait_highfive.hpp"
#include "requester/kinematics.hpp"
#include "test_helpers.hpp"

using namespace nikita_movement;

namespace {
constexpr int kMaxIterations = 200;
constexpr double kAngleTolerance = 1e-3;
constexpr double kHeadTolerance = 1e-3;
constexpr double kLiftThresholdDegrees = 5.0;
}  // namespace

class HighFiveGaitTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        rclcpp::NodeOptions options;
        auto overrides = test_helpers::defaultRobotParameters();
        options.parameter_overrides(overrides);

        node_ = std::make_shared<rclcpp::Node>("test_gait_highfive_node", options);
        actionPackagesParser_ = std::make_shared<CActionPackagesParser>(node_);
        kinematics_ = std::make_shared<CKinematics>(node_, actionPackagesParser_);
        params_ = test_helpers::makeDeclaredParameters(node_);
    }

    void TearDown() override {
        kinematics_.reset();
        actionPackagesParser_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPackagesParser> actionPackagesParser_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters params_;
};

TEST_F(HighFiveGaitTest, RaisesRightFrontLegAndReturns) {
    CHighFiveGait gait(node_, kinematics_, params_.highFive);
    const auto initialAngles = kinematics_->getAngles(ELegIndex::RightFront);
    const auto initialHead = kinematics_->getHead();

    gait.start(5.0, 0);
    geometry_msgs::msg::Twist twist;

    bool raised = false;
    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update(twist, CPose());
        const auto currentAngles = kinematics_->getAngles(ELegIndex::RightFront);
        if (currentAngles.femur_deg >= initialAngles.femur_deg + kLiftThresholdDegrees) {
            raised = true;
        }
    }

    EXPECT_TRUE(raised);
    EXPECT_LT(iterations, kMaxIterations);

    const auto finalAngles = kinematics_->getAngles(ELegIndex::RightFront);
    EXPECT_NEAR(finalAngles.coxa_deg, initialAngles.coxa_deg, kAngleTolerance);
    EXPECT_NEAR(finalAngles.femur_deg, initialAngles.femur_deg, kAngleTolerance);
    EXPECT_NEAR(finalAngles.tibia_deg, initialAngles.tibia_deg, kAngleTolerance);

    const auto finalHead = kinematics_->getHead();
    EXPECT_NEAR(finalHead.pitch_deg, initialHead.pitch_deg, kHeadTolerance);
    EXPECT_NEAR(finalHead.yaw_deg, initialHead.yaw_deg, kHeadTolerance);
}

TEST_F(HighFiveGaitTest, RequestStopReturnsToNeutralQuickly) {
    CHighFiveGait gait(node_, kinematics_, params_.highFive);
    const auto initialAngles = kinematics_->getAngles(ELegIndex::RightFront);

    gait.start(5.0, 0);
    geometry_msgs::msg::Twist twist;

    // Begin the raise phase for a few iterations.
    for (int i = 0; i < 3; ++i) {
        gait.update(twist, CPose());
    }

    gait.requestStop();

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update(twist, CPose());
    }

    EXPECT_LT(iterations, kMaxIterations);

    const auto finalAngles = kinematics_->getAngles(ELegIndex::RightFront);
    EXPECT_NEAR(finalAngles.coxa_deg, initialAngles.coxa_deg, kAngleTolerance);
    EXPECT_NEAR(finalAngles.femur_deg, initialAngles.femur_deg, kAngleTolerance);
    EXPECT_NEAR(finalAngles.tibia_deg, initialAngles.tibia_deg, kAngleTolerance);
}
