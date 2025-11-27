#include <gtest/gtest.h>

#include "nikita_utils/linear_interpolation.hpp"
#include "requester/interpolation.hpp"

TEST(LinearInterpolationPoseLike, InterpolatesOrientation) {
    const COrientation from(0.0, -10.0, 5.0);
    const COrientation to(10.0, 10.0, -5.0);

    const auto result = nikita_utils::linearInterpolate(from, to, 0.25);

    EXPECT_DOUBLE_EQ(result.roll, 2.5);
    EXPECT_DOUBLE_EQ(result.pitch, -5.0);
    EXPECT_DOUBLE_EQ(result.yaw, 2.5);
}

TEST(LinearInterpolationPoseLike, InterpolatesLegAngles) {
    const CLegAngles from(0.0, -20.0, 40.0);
    const CLegAngles to(10.0, 20.0, 10.0);

    const auto result = nikita_utils::linearInterpolate(from, to, 0.5);

    EXPECT_DOUBLE_EQ(result.coxa_deg, 5.0);
    EXPECT_DOUBLE_EQ(result.femur_deg, 0.0);
    EXPECT_DOUBLE_EQ(result.tibia_deg, 25.0);
}

TEST(LinearInterpolationPoseLike, InterpolatesPose) {
    const CPose from(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const CPose to(10.0, -10.0, 5.0, 20.0, -20.0, 10.0);

    const auto result = nikita_utils::linearInterpolate(from, to, 0.4);

    EXPECT_DOUBLE_EQ(result.position.x, 4.0);
    EXPECT_DOUBLE_EQ(result.position.y, -4.0);
    EXPECT_DOUBLE_EQ(result.position.z, 2.0);
    EXPECT_DOUBLE_EQ(result.orientation.roll, 8.0);
    EXPECT_DOUBLE_EQ(result.orientation.pitch, -8.0);
    EXPECT_DOUBLE_EQ(result.orientation.yaw, 4.0);
}
