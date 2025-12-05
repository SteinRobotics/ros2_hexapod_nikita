#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "requester/gait_bodyroll.hpp"
#include "requester/kinematics.hpp"

using namespace nikita_movement;

class BodyRollGaitTest : public ::testing::Test {
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
            rclcpp::Parameter("BODY_MAX_ROLL", 20.0),
            rclcpp::Parameter("BODY_MAX_PITCH", 10.0),
        });

        node_ = std::make_shared<rclcpp::Node>("test_gait_bodyroll_node", options);

        actionPackagesParser_ = std::make_shared<CActionPackagesParser>(node_);
        kinematics_ = std::make_shared<CKinematics>(node_, actionPackagesParser_);
        gait_ = std::make_unique<CBodyRollGait>(node_, kinematics_);
    }

    void TearDown() override {
        gait_.reset();
        kinematics_.reset();
        actionPackagesParser_.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPackagesParser> actionPackagesParser_;
    std::shared_ptr<CKinematics> kinematics_;
    std::unique_ptr<CBodyRollGait> gait_;
};

TEST_F(BodyRollGaitTest, StateTransitionsCoverAllStates) {
    // Initial state should be Stopped
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);

    // Start -> Starting
    gait_->start(0, 0);
    EXPECT_EQ(gait_->state(), EGaitState::Starting);

    // update until Running
    int max_iters = 1000;
    int iters = 0;
    geometry_msgs::msg::Twist vel;
    while (gait_->state() != EGaitState::Running && ++iters < max_iters) {
        gait_->update(vel, CPose());
    }
    EXPECT_EQ(gait_->state(), EGaitState::Running);
    EXPECT_LT(iters, max_iters);

    // requestStop -> StopPending
    gait_->requestStop();
    EXPECT_EQ(gait_->state(), EGaitState::StopPending);

    // cancelStop should return to Running
    gait_->cancelStop();
    EXPECT_EQ(gait_->state(), EGaitState::Running);

    // requestStop again and let it progress to Stopping and Stopped
    gait_->requestStop();
    EXPECT_EQ(gait_->state(), EGaitState::StopPending);

    // advance until Stopping
    iters = 0;
    while (gait_->state() != EGaitState::Stopping && ++iters < max_iters) {
        gait_->update(vel, CPose());
    }
    EXPECT_EQ(gait_->state(), EGaitState::Stopping);
    EXPECT_LT(iters, max_iters);

    // advance until final Stopped
    iters = 0;
    while (gait_->state() != EGaitState::Stopped && ++iters < max_iters) {
        gait_->update(vel, CPose());
    }
    EXPECT_EQ(gait_->state(), EGaitState::Stopped);
    EXPECT_LT(iters, max_iters);
}
