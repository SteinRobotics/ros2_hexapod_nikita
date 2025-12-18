#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using namespace nikita_movement;

class MoveBodyNewPublishTest : public ::testing::Test {
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

        node_ = std::make_shared<rclcpp::Node>("test_moveBodyNew_publish_node", options);
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

TEST_F(MoveBodyNewPublishTest, moveBodyNewPublishesJointState) {
    std::atomic<bool> received{false};
    sensor_msgs::msg::JointState::SharedPtr last_msg;

    auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            last_msg = msg;
            received.store(true);
        });

    // Prepare foot targets: use current positions so IK is reachable
    auto footTargets = kin_->getLegsPositions();
    CPose bodyPose = kin_->getCompleteBody().pose;

    // Call moveBodyNew which should publish a joint_state message once
    kin_->moveBodyNew(footTargets, bodyPose);

    // Spin the node a short while waiting for the message
    const auto timeout = 1000ms;
    auto start = std::chrono::steady_clock::now();
    while (!received.load() && (std::chrono::steady_clock::now() - start) < timeout) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(10ms);
    }

    EXPECT_TRUE(received.load());
    if (received.load()) {
        ASSERT_NE(last_msg, nullptr);
        // basic sanity checks: name and position lengths are multiples of 3 (3 joints per leg)
        EXPECT_EQ(last_msg->name.size() % 3, 0u);
        EXPECT_EQ(last_msg->position.size() % 3, 0u);
        EXPECT_EQ(last_msg->name.size(), last_msg->position.size());
    }
}
