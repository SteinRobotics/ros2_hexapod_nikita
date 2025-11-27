/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include "nikita_utils/linear_interpolation.hpp"
#include "requester/types.hpp"

namespace nikita_utils {

inline COrientation linearInterpolate(const COrientation& from, const COrientation& to, double t) {
    return {linearInterpolate(from.roll, to.roll, t), linearInterpolate(from.pitch, to.pitch, t),
            linearInterpolate(from.yaw, to.yaw, t)};
}

inline CLegAngles linearInterpolate(const CLegAngles& from, const CLegAngles& to, double t) {
    return {linearInterpolate(from.coxa_deg, to.coxa_deg, t),
            linearInterpolate(from.femur_deg, to.femur_deg, t),
            linearInterpolate(from.tibia_deg, to.tibia_deg, t)};
}

inline CPose linearInterpolate(const CPose& from, const CPose& to, double t) {
    CPose pose;
    pose.position.x = linearInterpolate(from.position.x, to.position.x, t);
    pose.position.y = linearInterpolate(from.position.y, to.position.y, t);
    pose.position.z = linearInterpolate(from.position.z, to.position.z, t);
    pose.orientation = linearInterpolate(from.orientation, to.orientation, t);
    return pose;
}

}  // namespace nikita_utils
