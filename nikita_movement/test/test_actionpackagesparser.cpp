#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/actionpackagesparser.hpp"
#include "test_helpers.hpp"

using namespace std;

class ActionPackagesParserTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_actionpackagesparser_node");
        parser_ = std::make_unique<CActionPackagesParser>(node_);
    }

    void TearDown() override {
        parser_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CActionPackagesParser> parser_;
};

TEST_F(ActionPackagesParserTest, NonexistentPackageReturnsEmpty) {
    auto& requests = parser_->getRequests("this_package_does_not_exist_12345");
    EXPECT_TRUE(requests.empty());
}
TEST_F(ActionPackagesParserTest, ExistingPackageLoadsCorrectly) {
    auto& requests = parser_->getRequests("STAND_UP");
    EXPECT_FALSE(requests.empty());
    EXPECT_EQ(requests.size(), 1);

    const auto& firstRequest = requests[0];
    EXPECT_TRUE(firstRequest.head.has_value());

    // The following checks depend on the YAML content for STAND_UP
    EXPECT_FALSE(firstRequest.body.has_value());
    EXPECT_TRUE(firstRequest.legAngles.has_value());
    EXPECT_FALSE(firstRequest.footPositions.has_value());

    EXPECT_DOUBLE_EQ(firstRequest.factorDuration, 1.0);

    const auto& head = firstRequest.head.value();
    EXPECT_DOUBLE_EQ(head.degYaw, 0.0);
    EXPECT_DOUBLE_EQ(head.degPitch, 0.0);

    const auto& legAngles = firstRequest.legAngles.value();
    // helper signature: expectAnglesNear(expected, actual, tol)
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::RightFront), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::RightBack), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::RightMid), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::LeftFront), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::LeftMid), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 2.276, 7.704), legAngles.at(ELegIndex::LeftBack), 1e-3);

    auto& requests2 = parser_->getRequests("LAYDOWN");
    EXPECT_FALSE(requests2.empty());
    EXPECT_EQ(requests2.size(), 1);
    const auto& firstRequest2 = requests2[0];
    EXPECT_TRUE(firstRequest2.head.has_value());
    // YAML for LAYDOWN currently does not include an explicit body entry, so it should be empty
    EXPECT_FALSE(firstRequest2.body.has_value());
    EXPECT_TRUE(firstRequest2.legAngles.has_value());
    EXPECT_FALSE(firstRequest2.footPositions.has_value());
    EXPECT_DOUBLE_EQ(firstRequest2.factorDuration, 1.0);
    const auto& head2 = firstRequest2.head.value();
    EXPECT_DOUBLE_EQ(head2.degYaw, 0.0);
    EXPECT_DOUBLE_EQ(head2.degPitch, -20.0);
    // no body to check for LAYDOWN
    const auto& legAngles2 = firstRequest2.legAngles.value();
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles2.at(ELegIndex::RightFront), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles2.at(ELegIndex::RightBack), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles2.at(ELegIndex::RightMid), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles2.at(ELegIndex::LeftFront), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles2.at(ELegIndex::LeftMid), 1e-3);
    expectAnglesNear(CLegAngles(0.0, 70.723, -53.320), legAngles2.at(ELegIndex::LeftBack), 1e-3);
}