#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "nikita_utils/filters.hpp"
#include "nikita_utils/geometry.hpp"
#include "nikita_utils/msg_twist.hpp"
#include "nikita_utils/simpletimer.hpp"
#include "requester/igaits.hpp"
#include "requester/types.hpp"

class CKinematics;

namespace nikita_movement {

class CTripodGait : public IGait {
   public:
    CTripodGait(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics,
                double leg_lift_height, double gait_step_length, double factor_velocity_to_gait_cycle_time);
    ~CTripodGait() override = default;

    void start() override;
    bool update(const geometry_msgs::msg::Twist& velocity, const CPose& body) override;
    void requestStop() override;
    void cancelStop() override;
    EGaitState state() const override {
        return state_;
    }

   private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CKinematics> kinematics_;
    double leg_lift_height_ = double(0);
    double gait_step_length_ = double(0);
    double factor_velocity_to_gait_cycle_time_ = double(0);
    double head_max_yaw_amplitude_ = double(0);
    EGaitState state_ = EGaitState::Stopped;

    bool use_group1_ = true;

    const std::vector<ELegIndex> group_first_tripod_ = {ELegIndex::LeftFront, ELegIndex::RightMid,
                                                        ELegIndex::LeftBack};
    const std::vector<ELegIndex> group_second_tripod_ = {ELegIndex::RightFront, ELegIndex::LeftMid,
                                                         ELegIndex::RightBack};

    double phase_ = double(0);
    bool is_start_phase_done_ = false;
    bool is_leave_cycle_requested_ = false;
    ELegIndex active_leg_index_ = ELegIndex::RightFront;

    CSimpleTimer no_velocity_timer_;

    geometry_msgs::msg::Twist velocity_{geometry_msgs::msg::Twist()};
};

}  // namespace nikita_movement
