#include <gtest/gtest.h>

#include <Graphics/Animation.hpp>

TEST(AdvanceWrapping, Steps)
{
    double value = 1.0;
    Graphics::AdvanceWrapping(value, 0.0, 4.0, 0.25);
    EXPECT_DOUBLE_EQ(value, 2.0);
}

TEST(AdvanceWrapping, WrapsPastMax)
{
    // 1 + 0.5 * 4 = 3, which is 1 past max, so it lands 1 past min.
    double value = 1.0;
    Graphics::AdvanceWrapping(value, -2.0, 2.0, 0.5);
    EXPECT_DOUBLE_EQ(value, -1.0);
}

TEST(AdvanceWrapping, FullTurnReturnsToStart)
{
    double value = 1.5;
    Graphics::AdvanceWrapping(value, 1.0, 3.0, 1.0);
    EXPECT_DOUBLE_EQ(value, 1.5);
}

TEST(AdvanceBouncing, Steps)
{
    double value = 1.0;
    double direction = -1.0;
    Graphics::AdvanceBouncing(value, direction, 0.0, 4.0, 0.125);
    EXPECT_DOUBLE_EQ(value, 0.5);
    EXPECT_DOUBLE_EQ(direction, -1.0);
}

TEST(AdvanceBouncing, ReflectsOffMax)
{
    // 3 + 0.5 * 4 = 5, which is 1 past max, so it reflects to 1 short of max.
    double value = 3.0;
    double direction = 1.0;
    Graphics::AdvanceBouncing(value, direction, 0.0, 4.0, 0.5);
    EXPECT_DOUBLE_EQ(value, 3.0);
    EXPECT_DOUBLE_EQ(direction, -1.0);
}

TEST(AdvanceBouncing, ReflectsOffMin)
{
    double value = 1.0;
    double direction = -1.0;
    Graphics::AdvanceBouncing(value, direction, 0.0, 4.0, 0.5);
    EXPECT_DOUBLE_EQ(value, 1.0);
    EXPECT_DOUBLE_EQ(direction, 1.0);
}
