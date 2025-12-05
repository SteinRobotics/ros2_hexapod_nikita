#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "requester/gait_laydown.hpp"
#include "requester/gait_standup.hpp"
#include "requester/kinematics.hpp"

using namespace nikita_movement;

namespace {
constexpr double kPositionTolerance = 0.06;
constexpr double kTravelRequirement = 0.05;
constexpr int kMaxIterations = 500;
}  // namespace

class VerticalGaitTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

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

        node_ = std::make_shared<rclcpp::Node>("test_gait_vertical_node", options);
        actionPackagesParser_ = std::make_shared<CActionPackagesParser>(node_);
        kinematics_ = std::make_shared<CKinematics>(node_, actionPackagesParser_);
    }

    void TearDown() override {
        kinematics_.reset();
        actionPackagesParser_.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPackagesParser> actionPackagesParser_;
    std::shared_ptr<CKinematics> kinematics_;
};

TEST_F(VerticalGaitTest, StandUpStopsAtStandingHeight) {
    const auto laydownTargets = kinematics_->getLegsLayDownPositions();
    const auto standingTargets = kinematics_->getLegsStandingPositions();

    // Ensure we start from laydown pose.
    kinematics_->moveBody(laydownTargets, CPose());
    const auto initialPositions = kinematics_->getLegsPositions();

    CStandUpGait gait(node_, kinematics_);
    EXPECT_EQ(gait.state(), EGaitState::Stopped);

    gait.start(3.0, 0);
    geometry_msgs::msg::Twist twist;

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update(twist, CPose());
    }

    EXPECT_EQ(gait.state(), EGaitState::Stopped);
    EXPECT_LT(iterations, kMaxIterations);

    const auto finalPositions = kinematics_->getLegsPositions();
    for (const auto& [legIndex, target] : standingTargets) {
        const auto& actual = finalPositions.at(legIndex);
        EXPECT_NEAR(actual.z, target.z, kPositionTolerance);
    }
}

TEST_F(VerticalGaitTest, LayDownStopsAtLaydownHeight) {
    const auto laydownTargets = kinematics_->getLegsLayDownPositions();
    const auto standingTargets = kinematics_->getLegsStandingPositions();

    // Ensure we start from standing pose.
    kinematics_->moveBodyNew(standingTargets, CPose());
    const auto initialPositions = kinematics_->getLegsPositions();

    CLayDownGait gait(node_, kinematics_);
    EXPECT_EQ(gait.state(), EGaitState::Stopped);

    gait.start(3.0, 0);
    geometry_msgs::msg::Twist twist;

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update(twist, CPose());
    }

    EXPECT_EQ(gait.state(), EGaitState::Stopped);
    EXPECT_LT(iterations, kMaxIterations);

    const auto finalPositions = kinematics_->getLegsPositions();
    for (const auto& [legIndex, target] : laydownTargets) {
        const auto& actual = finalPositions.at(legIndex);
        EXPECT_NEAR(actual.z, target.z, kPositionTolerance);
    }
}
