#include <gtest/gtest.h>

#include "handler/servohandler.hpp"
#include "mock/mock_servohandler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "requester/requester.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

class RequesterTest : public ::testing::Test {
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
            rclcpp::Parameter("STANDING_FOOT_POS_X",
                              std::vector<double>{0.092, 0.0, -0.092, 0.092, 0.0, -0.092}),
            rclcpp::Parameter("STANDING_FOOT_POS_Y",
                              std::vector<double>{0.092, 0.130, 0.092, -0.092, -0.130, -0.092}),
            rclcpp::Parameter("STANDING_FOOT_POS_Z",
                              std::vector<double>{-0.050, -0.050, -0.050, -0.050, -0.050, -0.050}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_X",
                              std::vector<double>{0.071, 0.0, -0.071, 0.071, 0.0, -0.071}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_Y",
                              std::vector<double>{0.071, 0.100, 0.071, -0.071, -0.100, -0.071}),
            rclcpp::Parameter("LAYDOWN_FOOT_POS_Z",
                              std::vector<double>{0.010, 0.010, 0.010, 0.010, 0.010, 0.010}),
        });
        node_ = std::make_shared<rclcpp::Node>("test_requester_node", options);
        servoHandlerMock_ = std::make_shared<CServoHandlerMock>(node_);
        requester_ = std::make_unique<CRequester>(node_, servoHandlerMock_);
    }

    void TearDown() override {
        requester_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CRequester> requester_;
    std::shared_ptr<CServoHandlerMock> servoHandlerMock_;
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

    EXPECT_NO_THROW(requester_->onMovementTypeRequest(msg));
}
