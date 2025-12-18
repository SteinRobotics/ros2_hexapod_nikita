#pragma once

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace nikita_movement {

struct Parameters {
    struct BodyRoll {
        double body_max_roll_deg{0.0};
        double body_max_pitch_deg{0.0};
    };

    struct Clap {};

    struct HighFive {};

    struct LayDown {};

    struct LegWave {
        double leg_lift_height{0.0};
    };

    struct Ripple {};

    struct Look {
        double body_max_yaw_deg{0.0};
        double head_max_yaw_deg{0.0};
    };

    struct StandUp {};

    struct TestLegs {
        double coxa_delta_deg{0.0};
        double femur_delta_deg{0.0};
        double tibia_delta_deg{0.0};
        double hold_time_per_leg{0.0};
    };

    struct Tripod {
        double head_amplitude_yaw_deg{0.0};
        double factor_velocity_to_gait_cycle_time{0.0};
        double gait_step_length{0.0};
        double leg_lift_height{0.0};
    };

    struct Waiting {
        double leg_lift_height{0.0};
    };

    struct Watch {
        double body_max_yaw_deg{0.0};
        double head_max_yaw_deg{0.0};
    };

    BodyRoll bodyRoll;
    Clap clap;
    HighFive highFive;
    LayDown layDown;
    LegWave legWave;
    Ripple ripple;
    Look look;
    StandUp standUp;
    TestLegs testLegs;
    Tripod tripod;
    Waiting waiting;
    Watch watch;

    static Parameters declare(std::shared_ptr<rclcpp::Node> node);
};

inline Parameters Parameters::declare(std::shared_ptr<rclcpp::Node> node) {
    Parameters params;

    // Generic Parameters
    const double leg_lift_height = node->declare_parameter<double>("LEG_LIFT_HEIGHT");

    // Body Roll
    params.bodyRoll.body_max_roll_deg = node->declare_parameter<double>("BODY_MAX_ROLL");
    params.bodyRoll.body_max_pitch_deg = node->declare_parameter<double>("BODY_MAX_PITCH");

    // Tripod
    params.tripod.head_amplitude_yaw_deg = node->declare_parameter<double>("HEAD_MAX_YAW_TRIPOD");
    params.tripod.factor_velocity_to_gait_cycle_time =
        node->declare_parameter<double>("FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME");
    params.tripod.gait_step_length = node->declare_parameter<double>("GAIT_STEP_LENGTH");
    params.tripod.leg_lift_height = leg_lift_height;

    // Waiting
    params.waiting.leg_lift_height = leg_lift_height;

    // Leg Wave
    params.legWave.leg_lift_height = node->declare_parameter<double>("GAIT_LEG_WAVE_LEG_LIFT_HEIGHT");
    params.look.body_max_yaw_deg = node->declare_parameter<double>("GAIT_LOOK_BODY_MAX_YAW");
    params.look.head_max_yaw_deg = node->declare_parameter<double>("GAIT_LOOK_HEAD_MAX_YAW");

    // Watch
    params.watch.body_max_yaw_deg = node->declare_parameter<double>("GAIT_WATCH_BODY_MAX_YAW");
    params.watch.head_max_yaw_deg = node->declare_parameter<double>("GAIT_WATCH_HEAD_MAX_YAW");

    // Test Legs
    params.testLegs.coxa_delta_deg = node->declare_parameter<double>("TESTLEGS_COXA_DELTA_DEG");
    params.testLegs.femur_delta_deg = node->declare_parameter<double>("TESTLEGS_FEMUR_DELTA_DEG");
    params.testLegs.tibia_delta_deg = node->declare_parameter<double>("TESTLEGS_TIBIA_DELTA_DEG");
    params.testLegs.hold_time_per_leg = node->declare_parameter<double>("TESTLEGS_HOLD_TIME_PER_LEG");

    return params;
}

}  // namespace nikita_movement
