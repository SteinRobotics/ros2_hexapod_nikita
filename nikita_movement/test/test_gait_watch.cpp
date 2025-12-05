#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "requester/gait_watch.hpp"
#include "requester/kinematics.hpp"

using namespace nikita_movement;

namespace {
constexpr int kMaxIterations = 400;
constexpr double kHeadSweepThreshold = 30.0;
constexpr double kBodySweepThreshold = 15.0;
constexpr double kTolerance = 1e-3;
}  // namespace

class WatchGaitTest : public ::testing::Test {
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

        node_ = std::make_shared<rclcpp::Node>("test_gait_watch_node", options);
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

TEST_F(WatchGaitTest, SweepsHeadAndBodyAndReturns) {
    CWatchGait gait(node_, kinematics_);
    const auto initialHead = kinematics_->getHead();
    const auto initialBody = kinematics_->getBody();

    gait.start(5.0, 0);
    geometry_msgs::msg::Twist twist;

    bool seenLeftHead = false;
    bool seenRightHead = false;
    bool seenLeftBody = false;
    bool seenRightBody = false;

    int iterations = 0;
    while (gait.state() != EGaitState::Stopped && iterations++ < kMaxIterations) {
        gait.update(twist, CPose());
        const auto head = kinematics_->getHead();
        const auto body = kinematics_->getBody();
        if (head.yaw_deg <= initialHead.yaw_deg - kHeadSweepThreshold) {
            seenLeftHead = true;
        }
        if (head.yaw_deg >= initialHead.yaw_deg + kHeadSweepThreshold) {
            seenRightHead = true;
        }
        if (body.orientation.yaw_deg <= initialBody.orientation.yaw_deg - kBodySweepThreshold) {
            seenLeftBody = true;
        }
        if (body.orientation.yaw_deg >= initialBody.orientation.yaw_deg + kBodySweepThreshold) {
            seenRightBody = true;
        }
    }

    EXPECT_TRUE(seenLeftHead);
    EXPECT_TRUE(seenRightHead);
    EXPECT_TRUE(seenLeftBody);
    EXPECT_TRUE(seenRightBody);
    EXPECT_LT(iterations, kMaxIterations);

    const auto finalHead = kinematics_->getHead();
    const auto finalBody = kinematics_->getBody();
    EXPECT_NEAR(finalHead.yaw_deg, initialHead.yaw_deg, kTolerance);
    EXPECT_NEAR(finalHead.pitch_deg, initialHead.pitch_deg, kTolerance);
    EXPECT_NEAR(finalBody.orientation.yaw_deg, initialBody.orientation.yaw_deg, kTolerance);
}

TEST_F(WatchGaitTest, RequestStopReturnsToInitialState) {
    CWatchGait gait(node_, kinematics_);
    const auto initialHead = kinematics_->getHead();
    const auto initialBody = kinematics_->getBody();

    gait.start(5.0, 0);
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

    const auto finalHead = kinematics_->getHead();
    const auto finalBody = kinematics_->getBody();
    EXPECT_NEAR(finalHead.yaw_deg, initialHead.yaw_deg, kTolerance);
    EXPECT_NEAR(finalHead.pitch_deg, initialHead.pitch_deg, kTolerance);
    EXPECT_NEAR(finalBody.orientation.yaw_deg, initialBody.orientation.yaw_deg, kTolerance);
}
