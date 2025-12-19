#pragma once

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

class CTripodGait : public IGait {
   public:
    CTripodGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                Parameters::Tripod& params);
    ~CTripodGait() override = default;

    void start(double duration_s, uint8_t direction) override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    Parameters::Tripod params_;
    EGaitState state_ = EGaitState::Stopped;

    const std::vector<ELegIndex> group_first_tripod_ = {ELegIndex::RightFront, ELegIndex::LeftMid,
                                                        ELegIndex::RightBack};
    const std::vector<ELegIndex> group_second_tripod_ = {ELegIndex::LeftFront, ELegIndex::RightMid,
                                                         ELegIndex::LeftBack};

    double phase_ = double(0);
    ELegIndex active_leg_index_ = ELegIndex::RightFront;

    CSimpleTimer no_velocity_timer_;
    std::map<ELegIndex, CPosition> target_positions_;
    CPose body_old_;

    geometry_msgs::msg::Twist velocity_{geometry_msgs::msg::Twist()};
};

}  // namespace nikita_movement
