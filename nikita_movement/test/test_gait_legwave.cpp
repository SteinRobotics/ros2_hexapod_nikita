#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "requester/gait_legwave.hpp"
#include "requester/kinematics.hpp"

using namespace nikita_movement;

namespace {
constexpr int kMaxIterations = 400;
constexpr double kLegLiftHeight = 0.025;
constexpr double kTolerance = 1e-3;
}  // namespace

class LegWaveGaitTest : public ::testing::Test {
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
            rclcpp::Parameter("LEG_LIFT_HEIGHT_LEGS_WAVE", kLegLiftHeight),

        });

        node_ = std::make_shared<rclcpp::Node>("test_gait_legwave_node", options);
        actionPackagesParser_ = std::make_shared<CActionPackagesParser>(node_);
        kinematics_ = std::make_shared<CKinematics>(node_, actionPackagesParser_);
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
};

TEST_F(LegWaveGaitTest, LiftOccursDuringRun) {
    CGaitLegWave gait(node_, kinematics_);
    const auto standing = kinematics_->getLegsStandingPositions();

    gait.start();
    geometry_msgs::msg::Twist twist;

    bool seen_lift = false;
    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update(twist, CPose());
        const auto pos = kinematics_->getLegsPositions();
        for (const auto& kv : standing) {
            const auto& idx = kv.first;
            const auto& base = kv.second;
            const auto& now = pos.at(idx);
            if (now.z >= base.z + (0.5 * kLegLiftHeight)) {
                seen_lift = true;
                break;
            }
        }
        if (seen_lift) break;
    }

    EXPECT_TRUE(seen_lift);
    EXPECT_LT(iterations, kMaxIterations);
}

TEST_F(LegWaveGaitTest, StopRequestReturnsToNeutral) {
    CGaitLegWave gait(node_, kinematics_);
    const auto standing = kinematics_->getLegsStandingPositions();

    gait.start();
    geometry_msgs::msg::Twist twist;

    for (int i = 0; i < 3; ++i) {
        gait.update(twist, CPose());
    }

    gait.requestStop();

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update(twist, CPose());
    }

    EXPECT_LT(iterations, kMaxIterations);

    const auto final_pos = kinematics_->getLegsPositions();
    for (const auto& kv : standing) {
        const auto& idx = kv.first;
        const auto& base = kv.second;
        const auto& now = final_pos.at(idx);
        EXPECT_NEAR(now.x, base.x, kTolerance);
        EXPECT_NEAR(now.y, base.y, kTolerance);
        EXPECT_NEAR(now.z, base.z, 1e-2);  // allow slightly larger tolerance for IK/clamping
    }
}
