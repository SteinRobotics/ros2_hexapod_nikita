#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

using namespace std;

class CompleteBodyInitTest : public ::testing::Test {
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
        node_ = std::make_shared<rclcpp::Node>("test_complete_body_init_node", options);
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

TEST_F(CompleteBodyInitTest, CompleteBodyLegsPresentAndLengthsSet) {
    const CBody& body = kin_->getCompleteBody();

    // Ensure legs map contains an entry for every ELegIndex and that link lengths match
    for (auto legIndex : magic_enum::enum_values<ELegIndex>()) {
        auto it = body.legs.find(legIndex);
        EXPECT_NE(it, body.legs.end())
            << "complete_body_.legs missing entry for " << magic_enum::enum_name(legIndex);
        if (it != body.legs.end()) {
            const CLegSegmentwise& seg = it->second;
            EXPECT_DOUBLE_EQ(seg.coxa.link.length_m, kin_->COXA_LENGTH);
            EXPECT_DOUBLE_EQ(seg.femur.link.length_m, kin_->FEMUR_LENGTH);
            EXPECT_DOUBLE_EQ(seg.tibia.link.length_m, kin_->TIBIA_LENGTH);
        }
    }
}
