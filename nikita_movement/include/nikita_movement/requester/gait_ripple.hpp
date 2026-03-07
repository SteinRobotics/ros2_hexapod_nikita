#pragma once

#include <array>
#include <geometry_msgs/msg/twist.hpp>
#include <magic_enum.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nikita_utils/filters.hpp"
#include "nikita_utils/geometry.hpp"
#include "nikita_utils/msg_twist.hpp"
#include "nikita_utils/simpletimer.hpp"
#include "requester/gait_parameters.hpp"
#include "requester/igaits.hpp"
#include "requester/kinematics.hpp"
#include "requester/types.hpp"

namespace nikita_movement {

class CRippleGait : public IGait {
   public:
    CRippleGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                Parameters::Ripple& params);
    ~CRippleGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body,
                const COrientation& head) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    // Ripple gait: 3 groups of 2 diagonal legs, each offset by 2π/3.
    // At any time, only 1 group (2 legs) is in the air, 4 legs on the ground.
    struct LegPhaseInfo {
        ELegIndex index;
        double phase_offset;
    };

    static constexpr double kTransferPhase = 2.0 * M_PI / 3.0;  // 1/3 of cycle: leg in air
    static constexpr double kSupportPhase = 4.0 * M_PI / 3.0;   // 2/3 of cycle: leg on ground

    // Wave from back to front, alternating sides
    static constexpr std::array<LegPhaseInfo, 6> leg_phases_ = {{
        {ELegIndex::RightBack, 0.0},
        {ELegIndex::LeftFront, 0.0},
        {ELegIndex::RightMid, 2.0 * M_PI / 3.0},
        {ELegIndex::LeftMid, 2.0 * M_PI / 3.0},
        {ELegIndex::RightFront, 4.0 * M_PI / 3.0},
        {ELegIndex::LeftBack, 4.0 * M_PI / 3.0},
    }};

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::Ripple params_;
    EGaitState state_ = EGaitState::Stopped;

    double phase_ = 0.0;

    CSimpleTimer no_velocity_timer_;
    std::map<ELegIndex, CPosition> target_positions_;
    CPose body_old_;

    geometry_msgs::msg::Twist velocity_{geometry_msgs::msg::Twist()};
};

}  // namespace nikita_movement
