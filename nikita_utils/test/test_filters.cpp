#include <gtest/gtest.h>

#include <nikita_utils/filters.hpp>

TEST(LowPassFilterTest, AlphaOnePassThrough) {
    double last = 1.0;
    double target = 5.0;
    double alpha = 1.0;  // full weight target
    double filtered = utils::lowPassFilter(last, target, alpha);
    EXPECT_DOUBLE_EQ(filtered, target);
}

TEST(LowPassFilterTest, AlphaZeroHoldLast) {
    double last = 2.5;
    double target = 9.0;
    double alpha = 0.0;  // full weight last
    double filtered = utils::lowPassFilter(last, target, alpha);
    EXPECT_DOUBLE_EQ(filtered, last);
}

TEST(LowPassFilterTest, MidAlphaBlend) {
    double last = 2.0;
    double target = 6.0;
    double alpha = 0.25;  // expect 0.25*6 + 0.75*2 = 1.5 + 1.5 = 3.0
    double filtered = utils::lowPassFilter(last, target, alpha);
    EXPECT_DOUBLE_EQ(filtered, 3.0);
}

TEST(LowPassFilterTest, ConvergesTowardTarget) {
    double value = 0.0;
    double target = 10.0;
    double alpha = 0.2;
    for (int i = 0; i < 25; ++i) {
        value = utils::lowPassFilter(value, target, alpha);
    }
    // After enough iterations should be close but not equal
    EXPECT_NEAR(value, 10.0, 0.5);
    EXPECT_LT(value, 10.0);  // still below target due to exponential approach
}

TEST(ChangeRateLimitTest, LimitsIncrease) {
    double last = 0.0;
    double target = 10.0;
    double changeRate = 2.0;  // max delta
    double limited = utils::filterChangeRate(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, last + changeRate);
}

TEST(ChangeRateLimitTest, LimitsDecrease) {
    double last = 10.0;
    double target = -5.0;
    double changeRate = 3.0;  // max delta
    double limited = utils::filterChangeRate(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, last - changeRate);
}

TEST(ChangeRateLimitTest, WithinRatePassesThrough) {
    double last = 5.0;
    double target = 6.5;      // delta 1.5
    double changeRate = 2.0;  // max delta greater than needed
    double limited = utils::filterChangeRate(last, target, changeRate);
    EXPECT_DOUBLE_EQ(limited, target);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
