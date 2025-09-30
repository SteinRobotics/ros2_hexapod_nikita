/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/kinematics.hpp"

#include <cmath>

using namespace std;

inline double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

CKinematics::CKinematics(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    for (const auto& [key, value] : legIndexToName) {
        legNameToIndex[value] = key;
    }

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

    INIT_FOOT_POS_X = node->declare_parameter<std::vector<double>>("INIT_FOOT_POS_X", std::vector<double>());
    INIT_FOOT_POS_Y = node->declare_parameter<std::vector<double>>("INIT_FOOT_POS_Y", std::vector<double>());
    INIT_FOOT_POS_Z = node->declare_parameter<std::vector<double>>("INIT_FOOT_POS_Z", std::vector<double>());

    STANDING_FOOT_POS_X =
        node->declare_parameter<std::vector<double>>("STANDING_FOOT_POS_X", std::vector<double>());
    STANDING_FOOT_POS_Y =
        node->declare_parameter<std::vector<double>>("STANDING_FOOT_POS_Y", std::vector<double>());
    STANDING_FOOT_POS_Z =
        node->declare_parameter<std::vector<double>>("STANDING_FOOT_POS_Z", std::vector<double>());

    LAYDOWN_FOOT_POS_X =
        node->declare_parameter<std::vector<double>>("LAYDOWN_FOOT_POS_X", std::vector<double>());
    LAYDOWN_FOOT_POS_Y =
        node->declare_parameter<std::vector<double>>("LAYDOWN_FOOT_POS_Y", std::vector<double>());
    LAYDOWN_FOOT_POS_Z =
        node->declare_parameter<std::vector<double>>("LAYDOWN_FOOT_POS_Z", std::vector<double>());

    BODY_MAX_ROLL = node->declare_parameter<double>("BODY_MAX_ROLL", rclcpp::PARAMETER_DOUBLE);
    BODY_MAX_PITCH = node->declare_parameter<double>("BODY_MAX_PITCH", rclcpp::PARAMETER_DOUBLE);
    BODY_MAX_YAW = node->declare_parameter<double>("BODY_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
    HEAD_MAX_YAW = node->declare_parameter<double>("HEAD_MAX_YAW", rclcpp::PARAMETER_DOUBLE);
    HEAD_MAX_PITCH = node->declare_parameter<double>("HEAD_MAX_PITCH", rclcpp::PARAMETER_DOUBLE);

    sqFemurLength_ = pow(FEMUR_LENGTH, 2);
    sqTibiaLength_ = pow(TIBIA_LENGTH, 2);

    // body center offsets
    for (uint32_t i = 0; i < LEG_NAMES.size(); i++) {
        auto& legName = LEG_NAMES.at(i);
        ELegIndex legIndex = legNameToIndex.at(legName);
        bodyCenterOffsets_[legIndex].x = CENTER_TO_COXA_X.at(i);
        bodyCenterOffsets_[legIndex].y = CENTER_TO_COXA_Y.at(i);
        bodyCenterOffsets_[legIndex].psi = OFFSET_COXA_ANGLE_DEG.at(i);
    }

    // Leg cootdinate system
    //            ^ x
    //       y <- |
    // z up -> +
    RCLCPP_INFO_STREAM(node_->get_logger(), "intialize legs current values");
    intializeLegs(legs_, INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z);
    RCLCPP_INFO_STREAM(node_->get_logger(), "intialize legs for standing position");
    intializeLegs(legsStanding_, STANDING_FOOT_POS_X, STANDING_FOOT_POS_Y, STANDING_FOOT_POS_Z);
    RCLCPP_INFO_STREAM(node_->get_logger(), "intialize legs for laydown position");
    intializeLegs(legsLayDown_, LAYDOWN_FOOT_POS_X, LAYDOWN_FOOT_POS_Y, LAYDOWN_FOOT_POS_Z);

    body_.position.x = 0.0;
    body_.position.y = 0.0;
    body_.position.z = 0.0;  // is this correct?
    body_.orientation.roll = 0.0;
    body_.orientation.pitch = 0.0;
    body_.orientation.yaw = 0.0;
}

void CKinematics::intializeLegs(std::map<ELegIndex, CLeg>& legs, std::vector<double>& posX,
                                std::vector<double>& posY, std::vector<double>& posZ) {
    for (uint32_t i = 0; i < LEG_NAMES.size(); i++) {
        auto& legName = LEG_NAMES.at(i);
        ELegIndex legIndex = legNameToIndex.at(legName);
        legs[legIndex] = CLeg();

        legs[legIndex].footPos_.x = posX.at(i);
        legs[legIndex].footPos_.y = posY.at(i);
        legs[legIndex].footPos_.z = posZ.at(i);

        // calc the legs[legIndex].angles_
        calcLegInverseKinematics(legs[legIndex].footPos_, legs[legIndex], legIndex);

        RCLCPP_INFO_STREAM(node_->get_logger(),
                           legIndexToName.at(legIndex)
                               << ": \tag: " << std::fixed << std::setprecision(0) << std::setw(3)
                               << legs[legIndex].angles_.degCoxa << "°, " << std::setw(3)
                               << legs[legIndex].angles_.degFemur << "°, " << std::setw(3)
                               << legs[legIndex].angles_.degTibia << "°\t| x: " << std::fixed
                               << std::setprecision(2) << std::setw(3) << legs[legIndex].footPos_.x
                               << ", y: " << std::setw(3) << legs[legIndex].footPos_.y
                               << ", z: " << std::setw(3) << legs[legIndex].footPos_.z);
    }
}

void CKinematics::moveBody(const std::map<ELegIndex, CPosition>& footTargets, const CPose body) {
    body_ = body;

    for (auto& [legIndex, footTarget] : footTargets) {
        auto& leg = legs_.at(legIndex);
        CPosition coxaPosition(bodyCenterOffsets_.at(legIndex).x, bodyCenterOffsets_.at(legIndex).y, 0.0);
        CPosition legBase = rotate(coxaPosition, body.orientation) + body.position;
        CPosition footRel = footTarget - legBase;
        calcLegInverseKinematics(footRel, leg, legIndex);

        RCLCPP_INFO_STREAM(node_->get_logger(),
                           legIndexToName.at(legIndex)
                               << ": \tag: " << std::fixed << std::setprecision(0) << std::setw(3)
                               << leg.angles_.degCoxa << "°, " << std::setw(3) << leg.angles_.degFemur
                               << "°, " << std::setw(3) << leg.angles_.degTibia << "°\t| x: " << std::fixed
                               << std::setprecision(2) << std::setw(3) << leg.footPos_.x
                               << ", y: " << std::setw(3) << leg.footPos_.y << ", z: " << std::setw(3)
                               << leg.footPos_.z);
    }
}

CPosition CKinematics::rotate(const CPosition& point, const COrientation& orientation) {
    double px = point.x;
    double py = point.y;
    double pz = point.z;

    // Rotation angles are in degrees, convert to radians
    double rollRad = deg2rad(orientation.roll);
    double pitchRad = deg2rad(orientation.pitch);
    double yawRad = deg2rad(orientation.yaw);

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
    leg.footPos_ = targetFeetPos;

    double agCoxaRad = atan2(targetFeetPos.x, targetFeetPos.y);
    double zOffset = COXA_HEIGHT - targetFeetPos.z;

    if (abs(cos(agCoxaRad)) < 0.00001) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "targetFeetPos.x = 0");
    }

    double lLegTopView = targetFeetPos.y / cos(agCoxaRad);  // L1

    double sqL = pow(zOffset, 2) + pow(lLegTopView - COXA_LENGTH, 2);
    double L = sqrt(sqL);

    double tmpFemur = (sqTibiaLength_ - sqFemurLength_ - sqL) / (-2 * FEMUR_LENGTH * L);
    if (abs(tmpFemur) > 1.0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "ERROR tmpFemur = " << tmpFemur);
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "targetFeetPos.x: " << targetFeetPos.x << " targetFeetPos.y: " << targetFeetPos.y
                                                << " targetFeetPos.z: " << targetFeetPos.z);
    }
    double agFemurRad = acos(zOffset / L) + acos(tmpFemur) - (M_PI / 2);

    double tmpTibia = (sqL - sqTibiaLength_ - sqFemurLength_) / (-2 * FEMUR_LENGTH * TIBIA_LENGTH);
    if (abs(tmpTibia) > 1.0) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "ERROR tmpTibia = " << tmpTibia);
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "targetFeetPos.x: " << targetFeetPos.x << " targetFeetPos.y: " << targetFeetPos.y
                                                << " targetFeetPos.z: " << targetFeetPos.z);
    }
    double agTibiaRad = acos(tmpTibia) - (M_PI / 2);

    leg.angles_.degCoxa = float(rad2deg(agCoxaRad - (M_PI / 2)));  // TODO why -90?
    leg.angles_.degFemur = float(rad2deg(agFemurRad));
    leg.angles_.degTibia = float(rad2deg(agTibiaRad));

    // for leg local coordinate system we have to add the body center offset
    leg.angles_.degCoxa += bodyCenterOffsets_[legIndex].psi;

    if (leg.angles_.degCoxa > 180.0) {
        leg.angles_.degCoxa -= 360.0;
    } else if (leg.angles_.degCoxa < -180.0) {
        leg.angles_.degCoxa += 360.0;
    }

    leg.footPos_.x += bodyCenterOffsets_[legIndex].x;
    leg.footPos_.y += bodyCenterOffsets_[legIndex].y;
}

void CKinematics::calcLegForwardKinematics(const CLegAngles target, CLeg& leg) {
    leg.angles_ = target;

    leg.footPos_.x = (COXA_LENGTH + FEMUR_LENGTH * cos(deg2rad(target.degFemur)) +
                      TIBIA_LENGTH * cos(deg2rad(-90 + target.degFemur + target.degTibia))) *
                     sin(deg2rad(target.degCoxa));

    leg.footPos_.y = (COXA_LENGTH + FEMUR_LENGTH * cos(deg2rad(target.degFemur)) +
                      TIBIA_LENGTH * cos(deg2rad(-90 + target.degFemur + target.degTibia))) *
                     cos(deg2rad(target.degCoxa));

    leg.footPos_.z = COXA_HEIGHT + (FEMUR_LENGTH * sin(deg2rad(target.degFemur)) +
                                    TIBIA_LENGTH * sin(deg2rad(-90 + target.degFemur + target.degTibia)));
}

CLeg& CKinematics::setCartesianFeet(const ELegIndex legIndex, const CPosition& targetFeetPos) {
    auto& leg = getLegs().at(legIndex);

    calcLegInverseKinematics(targetFeetPos, leg, legIndex);

    RCLCPP_INFO_STREAM(node_->get_logger(), "CKinematics::setCartesianFeet "
                                                << legIndexToName.at(legIndex) << ", x: " << std::fixed
                                                << std::setprecision(2) << leg.footPos_.x
                                                << ", y: " << leg.footPos_.y << ", z: " << leg.footPos_.z);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "CKinematics:: results to: " << legIndexToName.at(legIndex) << ", coxa: " << std::fixed
                                                    << std::setprecision(2) << leg.angles_.degCoxa
                                                    << ", femur: " << leg.angles_.degFemur
                                                    << ", tibia: " << leg.angles_.degTibia);

    return leg;
}

void CKinematics::setLegAngles(const ELegIndex index, const CLegAngles& angles) {
    auto& leg = legs_.at(index);

    RCLCPP_INFO_STREAM(node_->get_logger(), legIndexToName.at(index)
                                                << " before " << std::fixed << std::setprecision(2)
                                                << ":\tag: " << leg.angles_.degCoxa << "°, "
                                                << leg.angles_.degFemur << "°, " << leg.angles_.degTibia
                                                << "°\t| x: " << leg.footPos_.x << ", y: " << leg.footPos_.y
                                                << ", z: " << leg.footPos_.z);

    calcLegForwardKinematics(angles, leg);
    // leg.angles_.degCoxa += bodyCenterOffsets_[index].psi;   // guess this is wrong?!!
    leg.footPos_.x += bodyCenterOffsets_[index].x;
    leg.footPos_.y += bodyCenterOffsets_[index].y;

    RCLCPP_INFO_STREAM(node_->get_logger(), legIndexToName.at(index)
                                                << " after " << std::fixed << std::setprecision(2)
                                                << ":\tag: " << leg.angles_.degCoxa << "°, "
                                                << leg.angles_.degFemur << "°, " << leg.angles_.degTibia
                                                << "°\t| x: " << leg.footPos_.x << ", y: " << leg.footPos_.y
                                                << ", z: " << leg.footPos_.z);
}

CPose& CKinematics::getBody() {
    return body_;
}

CLeg& CKinematics::getLeg(ELegIndex index) {
    return legs_[index];
}

std::map<ELegIndex, CLeg>& CKinematics::getLegs() {
    return legs_;
}

CLegAngles& CKinematics::getAngles(ELegIndex index) {
    return legs_.at(index).angles_;
}

std::map<ELegIndex, CLegAngles> CKinematics::getLegsAngles() {
    std::map<ELegIndex, CLegAngles> legAngles;
    for (auto& [index, leg] : legs_) {
        legAngles[index] = leg.angles_;
    }
    return legAngles;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsPositions() const {
    std::map<ELegIndex, CPosition> positions;
    for (auto& [legIndex, leg] : legs_) {
        positions[legIndex] = leg.footPos_;
    }
    return positions;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsStandingPositions() const {
    std::map<ELegIndex, CPosition> footTargets;
    for (auto& [legIndex, leg] : legsStanding_) {
        footTargets[legIndex] = leg.footPos_;
    }
    return footTargets;
}

std::map<ELegIndex, CPosition> CKinematics::getLegsLayDownPositions() const {
    std::map<ELegIndex, CPosition> footTargets;
    for (auto& [legIndex, leg] : legsLayDown_) {
        footTargets[legIndex] = leg.footPos_;
    }
    return footTargets;
}