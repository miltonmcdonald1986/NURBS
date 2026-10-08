#include <gtest/gtest.h>

#include <NURBS/bernstein.hpp>

#include <cstddef>

namespace
{

template <typename S>
concept BernsteinInvocable = requires(std::size_t i, std::size_t n, S u) { NURBS::bernstein(i, n, u); };

} // namespace

TEST(Bernstein, Degree0)
{
    EXPECT_DOUBLE_EQ(NURBS::bernstein(0, 0, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(0, 0, 0.3), 1.0);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(0, 0, 1.0), 1.0);
}

TEST(Bernstein, Degree1)
{
    // B_{0,1}(u) = 1 - u, B_{1,1}(u) = u
    EXPECT_DOUBLE_EQ(NURBS::bernstein(0, 1, 0.25), 0.75);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(1, 1, 0.25), 0.25);
}

TEST(Bernstein, CubicDouble)
{
    EXPECT_DOUBLE_EQ(NURBS::bernstein(0, 3, 0.5), 1.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(1, 3, 0.5), 3.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(2, 3, 0.5), 3.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(3, 3, 0.5), 1.0 / 8.0);

    EXPECT_DOUBLE_EQ(NURBS::bernstein(0, 3, 0.25), 27.0 / 64.0);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(1, 3, 0.25), 27.0 / 64.0);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(2, 3, 0.25), 9.0 / 64.0);
    EXPECT_DOUBLE_EQ(NURBS::bernstein(3, 3, 0.25), 1.0 / 64.0);
}

TEST(Bernstein, CubicFloat)
{
    EXPECT_FLOAT_EQ(NURBS::bernstein(0, 3, 0.5f), 1.0f / 8.0f);
    EXPECT_FLOAT_EQ(NURBS::bernstein(1, 3, 0.5f), 3.0f / 8.0f);
    EXPECT_FLOAT_EQ(NURBS::bernstein(2, 3, 0.5f), 3.0f / 8.0f);
    EXPECT_FLOAT_EQ(NURBS::bernstein(3, 3, 0.5f), 1.0f / 8.0f);

    EXPECT_FLOAT_EQ(NURBS::bernstein(0, 3, 0.25f), 27.0f / 64.0f);
    EXPECT_FLOAT_EQ(NURBS::bernstein(1, 3, 0.25f), 27.0f / 64.0f);
    EXPECT_FLOAT_EQ(NURBS::bernstein(2, 3, 0.25f), 9.0f / 64.0f);
    EXPECT_FLOAT_EQ(NURBS::bernstein(3, 3, 0.25f), 1.0f / 64.0f);
}

TEST(Bernstein, Endpoints)
{
    constexpr std::size_t n = 4;
    for (std::size_t i = 0; i <= n; ++i)
    {
        EXPECT_DOUBLE_EQ(NURBS::bernstein(i, n, 0.0), i == 0 ? 1.0 : 0.0) << "i = " << i;
        EXPECT_DOUBLE_EQ(NURBS::bernstein(i, n, 1.0), i == n ? 1.0 : 0.0) << "i = " << i;
    }
}

TEST(Bernstein, PartitionOfUnity)
{
    constexpr std::size_t n = 5;
    for (const double u : {0.0, 0.1, 0.3, 0.5, 0.77, 1.0})
    {
        double sum = 0.0;
        for (std::size_t i = 0; i <= n; ++i)
            sum += NURBS::bernstein(i, n, u);
        EXPECT_NEAR(sum, 1.0, 1e-14) << "u = " << u;
    }
}

TEST(Bernstein, Symmetry)
{
    // B_{i,n}(u) = B_{n-i,n}(1 - u)
    constexpr std::size_t n = 4;
    constexpr double u = 0.3;
    for (std::size_t i = 0; i <= n; ++i)
        EXPECT_DOUBLE_EQ(NURBS::bernstein(i, n, u), NURBS::bernstein(n - i, n, 1.0 - u)) << "i = " << i;
}

TEST(Bernstein, Constexpr)
{
    static_assert(NURBS::bernstein(1, 2, 0.5) == 0.5);
    EXPECT_EQ(NURBS::bernstein(1, 2, 0.5), 0.5);
}

TEST(Bernstein, Constraints)
{
    static_assert(BernsteinInvocable<double>);
    static_assert(BernsteinInvocable<float>);
    static_assert(!BernsteinInvocable<int>);
}
