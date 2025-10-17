#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/requester.hpp"
#include "action/action_executor.hpp"
#include "mock/mock_action_executor.hpp"

using namespace std::chrono_literals;

class RequesterTest : public ::testing::Test {
  protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("LEG_NAMES", std::vector<std::string>{"RightFront", "RightMid", "RightBack", "LeftFront", "LeftMid", "LeftBack"}),
            rclcpp::Parameter("COXA_LENGTH", 0.050),
            rclcpp::Parameter("FEMUR_LENGTH", 0.063),
            rclcpp::Parameter("TIBIA_LENGTH", 0.099),
            rclcpp::Parameter("COXA_HEIGHT", 0.045),
            rclcpp::Parameter("CENTER_TO_COXA_X", std::vector<double>{0.109, 0.0, -0.109, 0.109, 0.0, -0.109}),
            rclcpp::Parameter("CENTER_TO_COXA_Y", std::vector<double>{0.068, 0.088, 0.068, -0.068, -0.088, -0.068}),
            rclcpp::Parameter("OFFSET_COXA_ANGLE_DEG", std::vector<double>{45.0, 90.0, 135.0, -45.0, -90.0, -135.0}),
            rclcpp::Parameter("STANDING_FOOT_POS_X", std::vector<double>{0.092, 0.0, -0.092, 0.092, 0.0, -0.092}),
            rclcpp::Parameter("STANDING_FOOT_POS_Y", std::vector<double>{0.092, 0.130, 0.092, -0.092, -0.130, -0.092}),
            rclcpp::Parameter("STANDING_FOOT_POS_Z", std::vector<double>{-0.050, -0.050, -0.050, -0.050, -0.050, -0.050}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_X", std::vector<double>{0.071, 0.0, -0.071, 0.071, 0.0, -0.071}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_Y", std::vector<double>{0.071, 0.100, 0.071, -0.071, -0.100, -0.071}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_Z", std::vector<double>{0.010, 0.010, 0.010, 0.010, 0.010, 0.010}),
        });
        node_ = std::make_shared<rclcpp::Node>("test_requester_node", options);
    actionExecutor_ = std::make_shared<CMockActionExecutor>(node_);
    requester_ = std::make_unique<CRequester>(node_, actionExecutor_);
    }

    void TearDown() override {
        requester_.reset();
        actionExecutor_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CMockActionExecutor> actionExecutor_;
    std::unique_ptr<CRequester> requester_;
};

TEST_F(RequesterTest, ConstructAndUpdate) {
    // basic smoke test: update should run without throwing
    EXPECT_NO_THROW(requester_->update(std::chrono::milliseconds(0)));
}

TEST_F(RequesterTest, HandleNoRequestMessage) {
    // Create an empty MovementRequest message with NO_REQUEST and pass it to the handler
    nikita_interfaces::msg::MovementRequest msg;
    msg.name = "";  // empty name indicates NO_REQUEST in the code path
    // MovementRequest uses the field 'type' to indicate the request kind
    msg.type = nikita_interfaces::msg::MovementRequest::NO_REQUEST;

    EXPECT_NO_THROW(requester_->onMovementRequest(msg));
}

TEST_F(RequesterTest, HandleLayDownRequest) {
    nikita_interfaces::msg::MovementRequest msg;
    msg.name = "LAYDOWN";
    msg.type = nikita_interfaces::msg::MovementRequest::LAYDOWN;
    // duration must be non-zero so the requester issues a duration request
    msg.duration_ms = 1000;

    // smoke test: ensure handler processes LAYDOWN without throwing
    EXPECT_NO_THROW(requester_->onMovementRequest(msg));

    // After processing, the mock executor should have recorded requests
    // Expect at least one legs request and a duration/send request
    EXPECT_GT(actionExecutor_->last_requested_count, 0u);
    // There should be at least one CRequestLegs in last_requests
    size_t legs_count = actionExecutor_->countOfType<CRequestLegs>();
    EXPECT_GE(legs_count, 1u);
    // There should be at least one duration/send request
    size_t duration_count = actionExecutor_->countOfType<CRequestSendDuration>();
    EXPECT_GE(duration_count, 1u);

    // Inspect the legs request payload: should contain 6 leg entries with reasonable angles
    auto legs_map = actionExecutor_->getLegAnglesMap();
    EXPECT_EQ(legs_map.size(), 6u);
    for (const auto &pair : legs_map) {
        const auto &angles = pair.second;
        EXPECT_NEAR(angles.degCoxa, 0.0, 1e-3) << "degCoxa out of range";
        EXPECT_NEAR(angles.degFemur, 10.0, 1e-3) << "degFemur too small for LAYDOWN";
        EXPECT_NEAR(angles.degTibia, 20.0, 1e-3) << "degTibia unexpectedly large for LAYDOWN";
    }

    // Duration: ensure there is a duration request recorded (value defaulting is handled in message type)
    auto duration_ms = actionExecutor_->getDurationMs();
    EXPECT_EQ(duration_ms, 1000u);

    // Head angles should be set to the laydown values (expected in requestLayDown)
    auto [headYaw, headPitch] = actionExecutor_->getHeadAngles();
    EXPECT_NEAR(headYaw, 0.0f, 0.1f);
    EXPECT_NEAR(headPitch, -20.0f, 0.1f);
}

