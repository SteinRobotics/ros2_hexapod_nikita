/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/kinematics.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

using namespace std;
using namespace utils;

#define LOG_KINEMATICS_ACTIVE true

CKinematics::CKinematics(std::shared_ptr<rclcpp::Node> node,
                         std::shared_ptr<CActionPackagesParser> actionPackagesParser)
    : node_(node), actionPackagesParser_(actionPackagesParser) {
    LEG_NAMES = node->declare_parameter<std::vector<std::string>>("LEG_NAMES", std::vector<std::string>());

    COXA_LENGTH = node->declare_parameter<double>("COXA_LENGTH", rclcpp::PARAMETER_DOUBLE);
    COXA_HEIGHT = node->declare_parameter<double>("COXA_HEIGHT", rclcpp::PARAMETER_DOUBLE);
    FEMUR_LENGTH = node->declare_parameter<double>("FEMUR_LENGTH", rclcpp::PARAMETER_DOUBLE);
    TIBIA_LENGTH = node->declare_parameter<double>("TIBIA_LENGTH", rclcpp::PARAMETER_DOUBLE);

    CENTER_TO_COXA_X =
        node->declare_parameter<std::vector<double>>("CENTER_TO_COXA_X", std::vector<double>());
    CENTER_TO_COXA_Y =
        node->declare_parameter<std::vector<double>>("CENTER_TO_COXA_Y", std::vector<double>());
    OFFSET_COXA_ANGLE_DEG =
        node->declare_parameter<std::vector<double>>("OFFSET_COXA_ANGLE_DEG", std::vector<double>());

    BODY_MAX_ROLL = node->declare_parameter<double>("BODY_MAX_ROLL", rclcpp::PARAMETER_DOUBLE);
    BODY_MAX_PITCH = node->declare_parameter<double>("BODY_MAX_PITCH", rclcpp::PARAMETER_DOUBLE);
    BODY_MAX_YAW = node->declare_parameter<double>("BODY_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
    HEAD_MAX_YAW = node->declare_parameter<double>("HEAD_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
    HEAD_MAX_PITCH = node->declare_parameter<double>("HEAD_MAX_PITCH", rclcpp::PARAMETER_DOUBLE);

    sq_femur_length_ = pow(FEMUR_LENGTH, 2);
    sq_tibia_length_ = pow(TIBIA_LENGTH, 2);

    RCLCPP_INFO_STREAM(node_->get_logger(), "Kinematics before bodyCenterOffsets_:");

    // body center offsets
    for (uint32_t i = 0; i < LEG_NAMES.size(); i++) {
        auto& leg_name = LEG_NAMES.at(i);
        ELegIndex leg_index = legNameToIndex(leg_name);
        bodyCenterOffsets_[leg_index].x = CENTER_TO_COXA_X.at(i);
        bodyCenterOffsets_[leg_index].y = CENTER_TO_COXA_Y.at(i);
        bodyCenterOffsets_[leg_index].psi_deg = OFFSET_COXA_ANGLE_DEG.at(i);
    }

    auto foot_positions_standing = actionPackagesParser_->getFootPositions("footPositions_standing");
    initializeLegsNew(foot_positions_standing, body_, legs_);
    initializeLegsNew(foot_positions_standing, body_, legsStanding_);

    auto foot_positions_laying = actionPackagesParser_->getFootPositions("footPositions_laydown");
    initializeLegsNew(foot_positions_laying, body_, legsLayDown_);

    // initialize complete_body_ with entries for all legs and copy offsets
    complete_body_.pose = CPose();
    for (auto leg_index : magic_enum::enum_values<ELegIndex>()) {
        CLegSegmentwise seg;
        seg.coxa.link.length_m = COXA_LENGTH;
        seg.femur.link.length_m = FEMUR_LENGTH;
        seg.tibia.link.length_m = TIBIA_LENGTH;
        // initialize coxa offset from configured body center offsets
        seg.coxa.desc.offset_rad = deg2rad(bodyCenterOffsets_.at(leg_index).psi_deg);
        complete_body_.legs[leg_index] = seg;
    }
    // TODO adapt bodyCenterOffsets_ to new structure
    for (auto leg_index : magic_enum::enum_values<ELegIndex>()) {
        auto& offset = complete_body_.bodyCenterOffsets[leg_index];
        offset.x = bodyCenterOffsets_.at(leg_index).x;
        offset.y = bodyCenterOffsets_.at(leg_index).y;
        offset.psi_deg = bodyCenterOffsets_.at(leg_index).psi_deg;
    }

    // Initialize joint state publisher
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
}

void CKinematics::logLegsPositions(std::map<ELegIndex, CLeg>& legs) {
    if (!LOG_KINEMATICS_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(), "----------------------------------------");
    for (const auto& [index, leg] : legs) {
        logLegPosition(index, leg);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "----------------------------------------");
}

void CKinematics::logLegPosition(const ELegIndex index, const CLeg& leg) {
    if (!LOG_KINEMATICS_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       magic_enum::enum_name(index)
                           << ": \tag: " << std::fixed << std::setprecision(3) << std::setw(3)
                           << leg.angles_deg_.coxa_deg << "°, " << std::setw(3) << leg.angles_deg_.femur_deg
                           << "°, " << std::setw(3) << leg.angles_deg_.tibia_deg << "°\t| x: " << std::fixed
                           << std::setprecision(3) << std::setw(3) << leg.foot_pos_.x << ", y: "
                           << std::setw(3) << leg.foot_pos_.y << ", z: " << std::setw(3) << leg.foot_pos_.z);
}

void CKinematics::logHeadPosition() {
    if (!LOG_KINEMATICS_ACTIVE) return;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Head: \tYaw: " << std::fixed << std::setprecision(3) << std::setw(3) << head_.yaw_deg
                                       << "°, Pitch: " << std::setw(3) << head_.pitch_deg << "°");
}

void CKinematics::initializeLegsNew(const std::map<ELegIndex, CPosition>& footTargets, const CPose body,
                                    std::map<ELegIndex, CLeg>& legs) {
    for (const auto& [leg_index, foot_target] : footTargets) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initializing leg "
                                                     << magic_enum::enum_name(leg_index)
                                                     << " to foot target position x: " << foot_target.x
                                                     << ", y: " << foot_target.y << ", z: " << foot_target.z);
        auto leg = CLeg();
        CPosition coxa_position(bodyCenterOffsets_.at(leg_index).x, bodyCenterOffsets_.at(leg_index).y, 0.0);
        CPosition leg_base = rotate(coxa_position, body.orientation) + body.position;
        CPosition foot_rel = foot_target - leg_base;
        calcLegInverseKinematics(foot_rel, leg, leg_index);
        legs[leg_index] = leg;
    }
    logLegsPositions(legs);
}

void CKinematics::moveBody(const std::map<ELegIndex, CPosition>& footTargets, const CPose body) {
    body_ = body;

    for (auto& [leg_index, foot_target] : footTargets) {
        auto& leg = legs_.at(leg_index);
        CPosition coxa_position(bodyCenterOffsets_.at(leg_index).x, bodyCenterOffsets_.at(leg_index).y, 0.0);
        CPosition leg_base = rotate(coxa_position, body.orientation) + body.position;
        CPosition foot_rel = foot_target - leg_base;
        calcLegInverseKinematics(foot_rel, leg, leg_index);
    }
    logLegsPositions(legs_);
    // publish joint values after computing inverse kinematics
    publishJointStates();
}

// New moveBody implementation using the segment-wise IK solver (solveIKSegmentwise).
// For each leg:
//  - compute the hip pose (position + orientation) in world/body coordinates
//  - ensure a CLegSegmentwise entry exists and is initialized with link lengths and offsets
//  - call solveIKSegmentwise(hip_pose, footTarget, legSeg)
//  - write back resulting joint angles (degrees) and foot position into the existing CLeg
void CKinematics::moveBodyNew(const std::map<ELegIndex, CPosition>& footTargets, const CPose body_pose) {
    complete_body_.pose = body_pose;

    for (const auto& [leg_index, foot_target] : footTargets) {
        // Prepare hip pose: compute hip base position (coxa joint position) in world/body frame
        CPosition coxa_position(complete_body_.bodyCenterOffsets.at(leg_index).x,
                                complete_body_.bodyCenterOffsets.at(leg_index).y, 0.0);
        CPosition leg_base = rotate(coxa_position, body_pose.orientation) + body_pose.position;
        CPose hip_pose;
        hip_pose.position = leg_base;
        hip_pose.orientation = body_pose.orientation;

        // Ensure an entry for this leg in complete_body_.legs exists and is initialized
        CLegSegmentwise& leg_seg = complete_body_.legs[leg_index];

        // initialize geometry if missing (lengths)
        if (leg_seg.coxa.link.length_m == 0.0) {
            leg_seg.coxa.link.length_m = COXA_LENGTH;
        }
        if (leg_seg.femur.link.length_m == 0.0) {
            leg_seg.femur.link.length_m = FEMUR_LENGTH;
        }
        if (leg_seg.tibia.link.length_m == 0.0) {
            leg_seg.tibia.link.length_m = TIBIA_LENGTH;
        }

        // Set coxa offset from body center offsets (psi) if not already set
        // complete_body_.psi is stored in degrees
        double psi_deg = complete_body_.bodyCenterOffsets.at(leg_index).psi_deg;
        leg_seg.coxa.desc.offset_rad = deg2rad(psi_deg);

        // Call segment-wise IK: footTarget is in world/body coordinates
        bool within_limits = solveIKSegmentwise(hip_pose, foot_target, leg_seg);

        // Map results back into the simple CLeg structure used by the rest of the system
        auto& leg = legs_.at(leg_index);

        // coxa: state.angle_rad is stored without the desc.offset_rad (see solveIKSegmentwise mapping)
        // earlier code and tests agree on the mapping: CLegAngles.coxa_deg == rad2deg(state.angle_rad) + OFFSET_COXA_ANGLE_DEG[idx]
        double coxa_state_rad = leg_seg.coxa.state.angle_rad;
        double femur_state_rad = leg_seg.femur.state.angle_rad;
        double tibia_state_rad = leg_seg.tibia.state.angle_rad;

        CLegAngles outAngles;
        outAngles.coxa_deg = float(rad2deg(coxa_state_rad) + psi_deg);
        outAngles.femur_deg = float(rad2deg(femur_state_rad));
        outAngles.tibia_deg = float(rad2deg(tibia_state_rad));

        leg.angles_deg_ = outAngles;

        // Compute the actual foot position from the (possibly clamped) segment-wise
        // joint states and write it into the simple CLeg structure. This keeps
        // `legs_` consistent with `complete_body_.legs` after IK.
        CPosition actual_foot = computeFootPositionSegmentwise(hip_pose, leg_seg);
        leg.foot_pos_ = actual_foot;

        // Optionally log a warning if IK had to clamp
        if (!within_limits) {
            RCLCPP_WARN_STREAM(node_->get_logger(), "moveBodyNew: IK result for "
                                                        << magic_enum::enum_name(leg_index)
                                                        << " required clamping or was out of range");
        }
    }

    logLegsPositions(legs_);
    // publish joint values after computing inverse kinematics (mirror behavior of moveBody)
    publishJointStates();
}

void CKinematics::publishJointStates() {
    if (!joint_state_pub_) return;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();

    // Reserve space: each leg has 3 joints
    msg.name.reserve(legs_.size() * 3);
    msg.position.reserve(legs_.size() * 3);

    for (const auto& [leg_index, leg] : legs_) {
        std::string leg_name = std::string(magic_enum::enum_name(leg_index));
        msg.name.push_back(leg_name + "_coxa_joint");
        msg.name.push_back(leg_name + "_femur_joint");
        msg.name.push_back(leg_name + "_tibia_joint");

        // publish positions in radians
        msg.position.push_back(deg2rad(leg.angles_deg_.coxa_deg));
        msg.position.push_back(deg2rad(leg.angles_deg_.femur_deg));
        msg.position.push_back(deg2rad(leg.angles_deg_.tibia_deg));
    }

    // Publisher stored as PublisherBase::SharedPtr; cast back to the concrete
    // publisher type to call publish(). If cast fails, skip publishing.
    auto typed_pub =
        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(joint_state_pub_);
    if (typed_pub) {
        typed_pub->publish(msg);
    }
}

CPosition CKinematics::rotate(const CPosition& point, const COrientation& orientation) {
    double px = point.x;
    double py = point.y;
    double pz = point.z;

    // Rotation angles are in degrees, convert to radians
    double rollRad = deg2rad(orientation.roll_deg);
    double pitchRad = deg2rad(orientation.pitch_deg);
    double yawRad = deg2rad(orientation.yaw_deg);

    double cosRoll = cos(rollRad);
    double sinRoll = sin(rollRad);
    double cosPitch = cos(pitchRad);
    double sinPitch = sin(pitchRad);
    double cosYaw = cos(yawRad);
    double sinYaw = sin(yawRad);

    // Standard ZYX (yaw-pitch-roll) rotation matrix
    double rotatedX = cosYaw * cosPitch * px + (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) * py +
                      (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll) * pz;

    double rotatedY = sinYaw * cosPitch * px + (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) * py +
                      (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll) * pz;

    double rotatedZ = -sinPitch * px + cosPitch * sinRoll * py + cosPitch * cosRoll * pz;

    return {rotatedX, rotatedY, rotatedZ};
}

void CKinematics::calcLegInverseKinematics(const CPosition& targetFeetPos, CLeg& leg,
                                           const ELegIndex& legIndex) {
    leg.foot_pos_ = targetFeetPos;

    double agCoxaRad = atan2(targetFeetPos.x, targetFeetPos.y);
    double zOffset = COXA_HEIGHT - targetFeetPos.z;

    if (abs(cos(agCoxaRad)) < 0.00001) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "targetFeetPos.x = 0");
    }

    double lLegTopView = targetFeetPos.y / cos(agCoxaRad);  // L1

    double sqL = pow(zOffset, 2) + pow(lLegTopView - COXA_LENGTH, 2);
    double L = sqrt(sqL);

    double tmpFemur = (sq_tibia_length_ - sq_femur_length_ - sqL) / (-2 * FEMUR_LENGTH * L);
    if (abs(tmpFemur) > 1.0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "ERROR tmpFemur = " << tmpFemur);
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "targetFeetPos.x: " << targetFeetPos.x << " targetFeetPos.y: " << targetFeetPos.y
                                                << " targetFeetPos.z: " << targetFeetPos.z);
    }
    double agFemurRad = acos(zOffset / L) + acos(tmpFemur) - (M_PI / 2);

    double tmpTibia = (sqL - sq_tibia_length_ - sq_femur_length_) / (-2 * FEMUR_LENGTH * TIBIA_LENGTH);
    if (abs(tmpTibia) > 1.0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "ERROR tmpTibia = " << tmpTibia);
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "targetFeetPos.x: " << targetFeetPos.x << " targetFeetPos.y: " << targetFeetPos.y
                                                << " targetFeetPos.z: " << targetFeetPos.z);
    }
    double agTibiaRad = acos(tmpTibia) - (M_PI / 2);

    leg.angles_deg_.coxa_deg = float(rad2deg(agCoxaRad - (M_PI / 2)));  // TODO why -90?
    leg.angles_deg_.femur_deg = float(rad2deg(agFemurRad));
    leg.angles_deg_.tibia_deg = float(rad2deg(agTibiaRad));

    // for leg local coordinate system we have to add the body center offset
    leg.angles_deg_.coxa_deg += bodyCenterOffsets_[legIndex].psi_deg;

    if (leg.angles_deg_.coxa_deg > 180.0) {
        leg.angles_deg_.coxa_deg -= 360.0;
    } else if (leg.angles_deg_.coxa_deg < -180.0) {
        leg.angles_deg_.coxa_deg += 360.0;
    }

    leg.foot_pos_.x += bodyCenterOffsets_[legIndex].x;
    leg.foot_pos_.y += bodyCenterOffsets_[legIndex].y;
}

void CKinematics::calcLegForwardKinematics(const CLegAngles target, CLeg& leg) {
    leg.angles_deg_ = target;

    leg.foot_pos_.x = (COXA_LENGTH + FEMUR_LENGTH * cos(deg2rad(target.femur_deg)) +
                       TIBIA_LENGTH * cos(deg2rad(-90 + target.femur_deg + target.tibia_deg))) *
                      sin(deg2rad(90 + target.coxa_deg));

    leg.foot_pos_.y = (COXA_LENGTH + FEMUR_LENGTH * cos(deg2rad(target.femur_deg)) +
                       TIBIA_LENGTH * cos(deg2rad(-90 + target.femur_deg + target.tibia_deg))) *
                      cos(deg2rad(90 + target.coxa_deg));

    leg.foot_pos_.z = COXA_HEIGHT + (FEMUR_LENGTH * sin(deg2rad(target.femur_deg)) +
                                     TIBIA_LENGTH * sin(deg2rad(-90 + target.femur_deg + target.tibia_deg)));
}

void CKinematics::setSingleFeet(const ELegIndex legIndex, const CPosition& targetFeetPos) {
    moveBody({{legIndex, targetFeetPos}}, body_);
}

void CKinematics::setLegAngles(const ELegIndex index, const CLegAngles& angles) {
    auto& leg = legs_.at(index);

    // transform to leg coordinate system by subtracting the angle psi
    CLegAngles targetAngles = angles;
    targetAngles.coxa_deg -= bodyCenterOffsets_[index].psi_deg;

    calcLegForwardKinematics(targetAngles, leg);

    // transform back to robot coordinate system by adding body center offset and adding psi
    leg.foot_pos_.x += bodyCenterOffsets_[index].x;
    leg.foot_pos_.y += bodyCenterOffsets_[index].y;
    leg.angles_deg_.coxa_deg += bodyCenterOffsets_[index].psi_deg;

    logLegPosition(index, leg);
}

void CKinematics::setHead(CHead head) {
    head_ = head;
    logHeadPosition();
}

void CKinematics::setHead(double yaw_deg, double pitch_deg) {
    head_.yaw_deg = yaw_deg;
    head_.pitch_deg = pitch_deg;
    logHeadPosition();
}

std::map<ELegIndex, CLeg>& CKinematics::getLegs() {
    return legs_;
}

CLegAngles& CKinematics::getAngles(ELegIndex index) {
    return legs_.at(index).angles_deg_;
}

std::map<ELegIndex, CLegAngles> CKinematics::getLegsAngles() {
    std::map<ELegIndex, CLegAngles> legAngles;
    for (auto& [index, leg] : legs_) {
        legAngles[index] = leg.angles_deg_;
    }
    return legAngles;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsPositions() const {
    std::map<ELegIndex, CPosition> positions;
    for (auto& [legIndex, leg] : legs_) {
        positions[legIndex] = leg.foot_pos_;
    }
    return positions;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsStandingPositions() const {
    std::map<ELegIndex, CPosition> footTargets;
    for (auto& [legIndex, leg] : legsStanding_) {
        footTargets[legIndex] = leg.foot_pos_;
    }
    return footTargets;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsLayDownPositions() const {
    std::map<ELegIndex, CPosition> footTargets;
    for (auto& [legIndex, leg] : legsLayDown_) {
        footTargets[legIndex] = leg.foot_pos_;
    }
    return footTargets;
}

const CBody& CKinematics::getCompleteBody() const {
    return complete_body_;
}

// ------------------- new functions -----------------

// Compute forward kinematics for a 3-segment leg (coxa, femur, tibia)
// hip_base: position of the coxa joint in body coordinates (or world coordinates as appropriate)
// leg: segments where each joint.state.angle_rad and desc.offset_rad are in radians
// Conventions used:
// - Coxa rotates around the vertical axis (yaw) and controls azimuth
// - Femur and tibia rotate in the leg plane (pitch)
// - Link lengths are along the local extension direction of each segment
// Returned position is the foot position (x forward, y left, z up).
CPosition computeFootPositionSegmentwise(const CPosition& hip_base, const CLegSegmentwise& leg) {
    const double coxa_angle_rad = leg.coxa.state.angle_rad + leg.coxa.desc.offset_rad;
    const double femur_angle_rad = leg.femur.state.angle_rad + leg.femur.desc.offset_rad;
    const double tibia_angle_rad = leg.tibia.state.angle_rad + leg.tibia.desc.offset_rad;

    const double coxa_length = leg.coxa.link.length_m;
    const double femur_length = leg.femur.link.length_m;
    const double tibia_length = leg.tibia.link.length_m;

    const double cos_coxa = std::cos(coxa_angle_rad);
    const double sin_coxa = std::sin(coxa_angle_rad);
    const double cos_femur = std::cos(femur_angle_rad);
    const double sin_femur = std::sin(femur_angle_rad);
    const double cos_femur_tibia = std::cos(femur_angle_rad + tibia_angle_rad);
    const double sin_femur_tibia = std::sin(femur_angle_rad + tibia_angle_rad);

    const double planar_extension = coxa_length + femur_length * cos_femur + tibia_length * cos_femur_tibia;

    CPosition foot;
    foot.x = hip_base.x + cos_coxa * planar_extension;
    foot.y = hip_base.y + sin_coxa * planar_extension;
    foot.z = hip_base.z - (femur_length * sin_femur + tibia_length * sin_femur_tibia);
    return foot;
}

// Overload: accept CPose (position + orientation). Compute foot in hip-local frame then rotate by hip orientation
// and translate by hip position.
CPosition computeFootPositionSegmentwise(const CPose& hip_pose, const CLegSegmentwise& leg) {
    CPosition local = computeFootPositionSegmentwise(CPosition(0.0, 0.0, 0.0), leg);
    // local currently computed relative to hip at (0,0,0). Rotate by hip orientation (roll->pitch->yaw)
    const double roll = hip_pose.orientation.roll_deg;
    const double pitch = hip_pose.orientation.pitch_deg;
    const double yaw = hip_pose.orientation.yaw_deg;

    // rotate local vector (apply roll, then pitch, then yaw)
    double local_after_roll_x = local.x;
    double local_after_roll_y = std::cos(roll) * local.y - std::sin(roll) * local.z;
    double local_after_roll_z = std::sin(roll) * local.y + std::cos(roll) * local.z;

    double local_after_pitch_x = std::cos(pitch) * local_after_roll_x + std::sin(pitch) * local_after_roll_z;
    double local_after_pitch_y = local_after_roll_y;
    double local_after_pitch_z = -std::sin(pitch) * local_after_roll_x + std::cos(pitch) * local_after_roll_z;

    double local_after_yaw_x = std::cos(yaw) * local_after_pitch_x - std::sin(yaw) * local_after_pitch_y;
    double local_after_yaw_y = std::sin(yaw) * local_after_pitch_x + std::cos(yaw) * local_after_pitch_y;
    double local_after_yaw_z = local_after_pitch_z;

    CPosition foot;
    foot.x = hip_pose.position.x + local_after_yaw_x;
    foot.y = hip_pose.position.y + local_after_yaw_y;
    foot.z = hip_pose.position.z + local_after_yaw_z;
    return foot;
}

// Inverse kinematics: given desired foot position in world/body frame (foot_pos) and the hip pose
// (position+orientation), compute the joint angles for the provided CLegSegmentwise and store them
// in leg.*.state.angle_rad (values are set such that computeFootPositionSegmentwise(hip_pose, leg)
// reproduces the target within tolerances).
// Returns true if the target is reachable and the computed angles are within joint limits. If the
// target is reachable but angles exceed limits they will be clamped and the function returns false.
bool solveIKSegmentwise(const CPose& hip_pose, const CPosition& foot_pos, CLegSegmentwise& leg,
                        double tolerance) {
    const double coxa_length = leg.coxa.link.length_m;
    const double femur_length = leg.femur.link.length_m;
    const double tibia_length = leg.tibia.link.length_m;

    // Vector from hip to foot in world/body frame
    const double hip_to_foot_x = foot_pos.x - hip_pose.position.x;
    const double hip_to_foot_y = foot_pos.y - hip_pose.position.y;
    // positive when foot is below hip (hip_z - foot_z)
    const double hip_to_foot_z = hip_pose.position.z - foot_pos.z;

    // horizontal distance from hip to foot (projection onto XY plane)
    const double horizontal_distance =
        std::sqrt(hip_to_foot_x * hip_to_foot_x + hip_to_foot_y * hip_to_foot_y);

    // distance along leg plane excluding coxa length (offset along leg direction)
    const double leg_plane_offset = horizontal_distance - coxa_length;

    // distance in the plane of femur/tibia (resulting 2D problem)
    const double leg_plane_distance =
        std::sqrt(leg_plane_offset * leg_plane_offset + hip_to_foot_z * hip_to_foot_z);

    // reachability checks
    if (leg_plane_distance > (femur_length + tibia_length) + tolerance) {
        return false;  // target too far
    }
    if (leg_plane_distance < std::fabs(femur_length - tibia_length) - tolerance) {
        return false;  // target too close (inside inner workspace)
    }

    // Coxa yaw angle (azimuth)
    const double coxa_yaw = std::atan2(hip_to_foot_y, hip_to_foot_x);

    // Two-link planar IK for femur (femur_angle) and tibia (tibia_rel_angle)
    const double leg_plane_distance2 = leg_plane_distance * leg_plane_distance;
    const double femur_len = femur_length;
    const double tibia_len = tibia_length;

    double cos_tibia_rel =
        (leg_plane_distance2 - femur_len * femur_len - tibia_len * tibia_len) / (2.0 * femur_len * tibia_len);
    cos_tibia_rel = std::clamp(cos_tibia_rel, -1.0, 1.0);
    // choose the 'elbow-down' solution (positive sine)
    double sin_tibia_rel = std::sqrt(std::max(0.0, 1.0 - cos_tibia_rel * cos_tibia_rel));
    double tibia_rel_angle = std::atan2(sin_tibia_rel, cos_tibia_rel);  // relative angle at tibia

    double femur_angle = std::atan2(hip_to_foot_z, leg_plane_offset) -
                         std::atan2(tibia_len * sin_tibia_rel, femur_len + tibia_len * cos_tibia_rel);

    // Map these into stored state values (state.angle_rad) which are stored without offsets
    const double coxa_state = coxa_yaw - leg.coxa.desc.offset_rad;
    const double femur_state = femur_angle - leg.femur.desc.offset_rad;
    const double tibia_state = tibia_rel_angle - leg.tibia.desc.offset_rad;

    // Apply limits (clamp). If any clamp happens, we'll report false while writing the clamped values.
    bool within_limits = true;

    auto clamp_and_check = [&](double val, const CJointDesc& desc) {
        double clamped = std::clamp(val, desc.limit_min_rad, desc.limit_max_rad);
        if (std::abs(clamped - val) > 1e-12) within_limits = false;
        return clamped;
    };

    leg.coxa.state.angle_rad = clamp_and_check(coxa_state, leg.coxa.desc);
    leg.femur.state.angle_rad = clamp_and_check(femur_state, leg.femur.desc);
    leg.tibia.state.angle_rad = clamp_and_check(tibia_state, leg.tibia.desc);

    leg.coxa.state.dirty = true;
    leg.femur.state.dirty = true;
    leg.tibia.state.dirty = true;

    return within_limits;
}
