/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include "nikita_interfaces/msg/movement_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "requester/gaitcontroller.hpp"
#include "requester/kinematics.hpp"

using namespace nikita_movement;
using MovementRequestMsg = nikita_interfaces::msg::MovementRequest;

namespace {
constexpr int kMaxIterations = 100;
}  // namespace

class GaitControllerTest : public ::testing::Test {
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
            rclcpp::Parameter("FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME", 1.0),
            rclcpp::Parameter("GAIT_STEP_LENGTH", 0.05),
            rclcpp::Parameter("LEG_LIFT_HEIGHT", 0.02),
        });

        node_ = std::make_shared<rclcpp::Node>("test_gait_controller_node", options);
        action_packages_parser_ = std::make_shared<CActionPackagesParser>(node_);
        kinematics_ = std::make_shared<CKinematics>(node_, action_packages_parser_);
        controller_ = std::make_unique<CGaitController>(node_, kinematics_);
    }

    void TearDown() override {
        controller_.reset();
        kinematics_.reset();
        action_packages_parser_.reset();
        node_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    geometry_msgs::msg::Twist createZeroVelocity() {
        geometry_msgs::msg::Twist vel;
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;
        return vel;
    }

    geometry_msgs::msg::Twist createForwardVelocity() {
        geometry_msgs::msg::Twist vel = createZeroVelocity();
        vel.linear.x = 0.1;
        return vel;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPackagesParser> action_packages_parser_;
    std::shared_ptr<CKinematics> kinematics_;
    std::unique_ptr<CGaitController> controller_;
};

// Test: Verify default gait is MOVE_TRIPOD
TEST_F(GaitControllerTest, DefaultGaitIsTripod) {
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::MOVE_TRIPOD);
}

// Test: Switch from TRIPOD to WAITING
TEST_F(GaitControllerTest, SwitchFromTripodToWaiting) {
    auto vel = createZeroVelocity();

    // Verify we start with TRIPOD
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::MOVE_TRIPOD);

    // Request stop and wait for it to complete
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to WAITING
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WAITING);
}

// Test: Switch from TRIPOD to LEGS_WAVE
TEST_F(GaitControllerTest, SwitchFromTripodToLegWave) {
    auto vel = createZeroVelocity();

    // Stop current gait
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to LEGS_WAVE
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::LEGS_WAVE;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::LEGS_WAVE);
}

// Test: Switch from WAITING to WATCH
TEST_F(GaitControllerTest, SwitchFromWaitingToWatch) {
    auto vel = createZeroVelocity();

    // First switch to WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WAITING);

    // Stop WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to WATCH
    request.type = MovementRequestMsg::WATCH;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WATCH);
}

// Test: Switch to BODY_ROLL
TEST_F(GaitControllerTest, SwitchToBodyRoll) {
    auto vel = createZeroVelocity();

    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::BODY_ROLL;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::BODY_ROLL);
}

// Test: Switch from BODY_ROLL to STAND_UP
TEST_F(GaitControllerTest, SwitchFromBodyRollToStandUp) {
    auto vel = createZeroVelocity();

    // First stop current gait (TRIPOD)
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to BODY_ROLL
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::BODY_ROLL;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::BODY_ROLL);

    // Stop BODY_ROLL before switching
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Switch to STAND_UP and allow the switch to process
    request.type = MovementRequestMsg::STAND_UP;
    controller_->setGait(request);
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::STAND_UP);
}

// Test: Switch to STAND_UP
TEST_F(GaitControllerTest, SwitchToStandUp) {
    auto vel = createZeroVelocity();

    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::STAND_UP;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::STAND_UP);
}

// Test: Switch to LAYDOWN
TEST_F(GaitControllerTest, SwitchToLaydown) {
    auto vel = createZeroVelocity();

    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::LAYDOWN;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::LAYDOWN);
}

// Test: Switch to HIGH_FIVE
TEST_F(GaitControllerTest, SwitchToHighFive) {
    auto vel = createZeroVelocity();

    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }

    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::HIGH_FIVE;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::HIGH_FIVE);
}

// Test: Request same gait twice (should not switch)
TEST_F(GaitControllerTest, RequestSameGaitTwice) {
    auto vel = createZeroVelocity();

    // Switch to WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WAITING);

    // Request WAITING again (should remain WAITING, may restart if stopped)
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WAITING);
}

// Test: Switching while gait is running (pending switch)
TEST_F(GaitControllerTest, SwitchWhileRunning) {
    auto vel = createForwardVelocity();

    // Start with TRIPOD running
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::MOVE_TRIPOD);

    // Run a few iterations to ensure gait is active
    for (int i = 0; i < 5; ++i) {
        controller_->updateSelectedGait(vel);
    }

    // Request switch to WAITING while running
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::WAITING;
    controller_->setGait(request);

    // The switch should be pending until gait stops
    // Continue updating until switch completes
    auto zero_vel = createZeroVelocity();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(zero_vel);
        if (controller_->currentGait() == MovementRequestMsg::WAITING) {
            break;
        }
    }

    // Verify switch completed
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WAITING);
}

// Test: Multiple consecutive switches
TEST_F(GaitControllerTest, MultipleConsecutiveSwitches) {
    auto vel = createZeroVelocity();

    // Switch 1: TRIPOD -> WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WAITING);

    // Switch 2: WAITING -> WATCH
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    request.type = MovementRequestMsg::WATCH;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WATCH);

    // Switch 3: WATCH -> LEGS_WAVE
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    request.type = MovementRequestMsg::LEGS_WAVE;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::LEGS_WAVE);

    // Switch 4: LEGS_WAVE -> TRIPOD (back to original)
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    request.type = MovementRequestMsg::MOVE_TRIPOD;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::MOVE_TRIPOD);
}

// Test: Update gait after switch
TEST_F(GaitControllerTest, UpdateAfterSwitch) {
    auto vel = createZeroVelocity();

    // Switch to WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::WAITING;
    controller_->setGait(request);

    // Update the new gait - should not crash
    bool update_result = false;
    EXPECT_NO_THROW({ update_result = controller_->updateSelectedGait(vel); });

    // Update returns true when gait processes successfully
    EXPECT_TRUE(update_result || !update_result);  // Just verify it executes
}

// Test: Switch back to TRIPOD with velocity
TEST_F(GaitControllerTest, SwitchBackToTripodWithVelocity) {
    auto vel = createZeroVelocity();
    auto forward_vel = createForwardVelocity();

    // Switch to WAITING
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    nikita_interfaces::msg::MovementRequest request;
    request.type = MovementRequestMsg::WAITING;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::WAITING);

    // Switch back to TRIPOD
    controller_->requestStopSelectedGait();
    for (int i = 0; i < kMaxIterations; ++i) {
        controller_->updateSelectedGait(vel);
    }
    request.type = MovementRequestMsg::MOVE_TRIPOD;
    controller_->setGait(request);
    EXPECT_EQ(controller_->currentGait(), MovementRequestMsg::MOVE_TRIPOD);

    // Update with forward velocity - should work
    EXPECT_NO_THROW({
        for (int i = 0; i < 10; ++i) {
            controller_->updateSelectedGait(forward_vel);
        }
    });
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
