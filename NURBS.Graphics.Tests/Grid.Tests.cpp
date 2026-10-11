#include <gtest/gtest.h>

#include "Grid.hpp"

TEST(PowerOfTenAtLeast, ExactPowers)
{
    EXPECT_DOUBLE_EQ(Graphics::Detail::PowerOfTenAtLeast(1.0), 1.0);
    EXPECT_DOUBLE_EQ(Graphics::Detail::PowerOfTenAtLeast(10.0), 10.0);
    EXPECT_DOUBLE_EQ(Graphics::Detail::PowerOfTenAtLeast(0.01), 0.01);
}

TEST(PowerOfTenAtLeast, RoundsUp)
{
    EXPECT_DOUBLE_EQ(Graphics::Detail::PowerOfTenAtLeast(0.5), 1.0);
    EXPECT_DOUBLE_EQ(Graphics::Detail::PowerOfTenAtLeast(7.0), 10.0);
    EXPECT_DOUBLE_EQ(Graphics::Detail::PowerOfTenAtLeast(10.0001), 100.0);
    EXPECT_DOUBLE_EQ(Graphics::Detail::PowerOfTenAtLeast(0.03), 0.1);
}
