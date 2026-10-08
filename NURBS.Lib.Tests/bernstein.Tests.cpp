#include <gtest/gtest.h>

#include <NURBS/bernstein.hpp>

#include <cstddef>

namespace
{

template <typename S>
concept BernsteinInvocable = requires(std::size_t i, std::size_t n, S u) { NURBS::Bernstein(i, n, u); };

template <typename S>
concept AllBernsteinInvocable = requires(std::size_t n, S u) { NURBS::AllBernstein(n, u); };

} // namespace

TEST(Bernstein, Degree0)
{
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(0, 0, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(0, 0, 0.3), 1.0);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(0, 0, 1.0), 1.0);
}

TEST(Bernstein, Degree1)
{
    // B_{0,1}(u) = 1 - u, B_{1,1}(u) = u
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(0, 1, 0.25), 0.75);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(1, 1, 0.25), 0.25);
}

TEST(Bernstein, CubicDouble)
{
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(0, 3, 0.5), 1.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(1, 3, 0.5), 3.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(2, 3, 0.5), 3.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(3, 3, 0.5), 1.0 / 8.0);

    EXPECT_DOUBLE_EQ(NURBS::Bernstein(0, 3, 0.25), 27.0 / 64.0);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(1, 3, 0.25), 27.0 / 64.0);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(2, 3, 0.25), 9.0 / 64.0);
    EXPECT_DOUBLE_EQ(NURBS::Bernstein(3, 3, 0.25), 1.0 / 64.0);
}

TEST(Bernstein, CubicFloat)
{
    EXPECT_FLOAT_EQ(NURBS::Bernstein(0, 3, 0.5f), 1.0f / 8.0f);
    EXPECT_FLOAT_EQ(NURBS::Bernstein(1, 3, 0.5f), 3.0f / 8.0f);
    EXPECT_FLOAT_EQ(NURBS::Bernstein(2, 3, 0.5f), 3.0f / 8.0f);
    EXPECT_FLOAT_EQ(NURBS::Bernstein(3, 3, 0.5f), 1.0f / 8.0f);

    EXPECT_FLOAT_EQ(NURBS::Bernstein(0, 3, 0.25f), 27.0f / 64.0f);
    EXPECT_FLOAT_EQ(NURBS::Bernstein(1, 3, 0.25f), 27.0f / 64.0f);
    EXPECT_FLOAT_EQ(NURBS::Bernstein(2, 3, 0.25f), 9.0f / 64.0f);
    EXPECT_FLOAT_EQ(NURBS::Bernstein(3, 3, 0.25f), 1.0f / 64.0f);
}

TEST(Bernstein, Endpoints)
{
    constexpr std::size_t n = 4;
    for (std::size_t i = 0; i <= n; ++i)
    {
        EXPECT_DOUBLE_EQ(NURBS::Bernstein(i, n, 0.0), i == 0 ? 1.0 : 0.0) << "i = " << i;
        EXPECT_DOUBLE_EQ(NURBS::Bernstein(i, n, 1.0), i == n ? 1.0 : 0.0) << "i = " << i;
    }
}

TEST(Bernstein, PartitionOfUnity)
{
    constexpr std::size_t n = 5;
    for (const double u : {0.0, 0.1, 0.3, 0.5, 0.77, 1.0})
    {
        double sum = 0.0;
        for (std::size_t i = 0; i <= n; ++i)
            sum += NURBS::Bernstein(i, n, u);
        EXPECT_NEAR(sum, 1.0, 1e-14) << "u = " << u;
    }
}

TEST(Bernstein, Symmetry)
{
    // B_{i,n}(u) = B_{n-i,n}(1 - u)
    constexpr std::size_t n = 4;
    constexpr double u = 0.3;
    for (std::size_t i = 0; i <= n; ++i)
        EXPECT_DOUBLE_EQ(NURBS::Bernstein(i, n, u), NURBS::Bernstein(n - i, n, 1.0 - u)) << "i = " << i;
}

TEST(Bernstein, Constexpr)
{
    static_assert(NURBS::Bernstein(1, 2, 0.5) == 0.5);
    EXPECT_EQ(NURBS::Bernstein(1, 2, 0.5), 0.5);
}

TEST(Bernstein, Constraints)
{
    static_assert(BernsteinInvocable<double>);
    static_assert(BernsteinInvocable<float>);
    static_assert(!BernsteinInvocable<int>);
}

TEST(AllBernstein, Degree0)
{
    const auto B = NURBS::AllBernstein(0, 0.3);
    ASSERT_EQ(B.size(), 1u);
    EXPECT_DOUBLE_EQ(B[0], 1.0);
}

TEST(AllBernstein, Degree1)
{
    // B_{0,1}(u) = 1 - u, B_{1,1}(u) = u
    const auto B = NURBS::AllBernstein(1, 0.25);
    ASSERT_EQ(B.size(), 2u);
    EXPECT_DOUBLE_EQ(B[0], 0.75);
    EXPECT_DOUBLE_EQ(B[1], 0.25);
}

TEST(AllBernstein, CubicDouble)
{
    const auto B = NURBS::AllBernstein(3, 0.5);
    ASSERT_EQ(B.size(), 4u);
    EXPECT_DOUBLE_EQ(B[0], 1.0 / 8.0);
    EXPECT_DOUBLE_EQ(B[1], 3.0 / 8.0);
    EXPECT_DOUBLE_EQ(B[2], 3.0 / 8.0);
    EXPECT_DOUBLE_EQ(B[3], 1.0 / 8.0);

    const auto C = NURBS::AllBernstein(3, 0.25);
    ASSERT_EQ(C.size(), 4u);
    EXPECT_DOUBLE_EQ(C[0], 27.0 / 64.0);
    EXPECT_DOUBLE_EQ(C[1], 27.0 / 64.0);
    EXPECT_DOUBLE_EQ(C[2], 9.0 / 64.0);
    EXPECT_DOUBLE_EQ(C[3], 1.0 / 64.0);
}

TEST(AllBernstein, CubicFloat)
{
    const auto B = NURBS::AllBernstein(3, 0.5f);
    ASSERT_EQ(B.size(), 4u);
    EXPECT_FLOAT_EQ(B[0], 1.0f / 8.0f);
    EXPECT_FLOAT_EQ(B[1], 3.0f / 8.0f);
    EXPECT_FLOAT_EQ(B[2], 3.0f / 8.0f);
    EXPECT_FLOAT_EQ(B[3], 1.0f / 8.0f);

    const auto C = NURBS::AllBernstein(3, 0.25f);
    ASSERT_EQ(C.size(), 4u);
    EXPECT_FLOAT_EQ(C[0], 27.0f / 64.0f);
    EXPECT_FLOAT_EQ(C[1], 27.0f / 64.0f);
    EXPECT_FLOAT_EQ(C[2], 9.0f / 64.0f);
    EXPECT_FLOAT_EQ(C[3], 1.0f / 64.0f);
}

TEST(AllBernstein, Endpoints)
{
    constexpr std::size_t n = 4;
    const auto B0 = NURBS::AllBernstein(n, 0.0);
    const auto B1 = NURBS::AllBernstein(n, 1.0);
    ASSERT_EQ(B0.size(), n + 1);
    ASSERT_EQ(B1.size(), n + 1);
    for (std::size_t i = 0; i <= n; ++i)
    {
        EXPECT_DOUBLE_EQ(B0[i], i == 0 ? 1.0 : 0.0) << "i = " << i;
        EXPECT_DOUBLE_EQ(B1[i], i == n ? 1.0 : 0.0) << "i = " << i;
    }
}

TEST(AllBernstein, PartitionOfUnity)
{
    constexpr std::size_t n = 5;
    for (const double u : {0.0, 0.1, 0.3, 0.5, 0.77, 1.0})
    {
        double sum = 0.0;
        for (const double b : NURBS::AllBernstein(n, u))
            sum += b;
        EXPECT_NEAR(sum, 1.0, 1e-14) << "u = " << u;
    }
}

TEST(AllBernstein, MatchesBernstein)
{
    for (std::size_t n = 0; n <= 6; ++n)
        for (const double u : {0.0, 0.1, 0.3, 0.5, 0.77, 1.0})
        {
            const auto B = NURBS::AllBernstein(n, u);
            ASSERT_EQ(B.size(), n + 1);
            for (std::size_t i = 0; i <= n; ++i)
                EXPECT_NEAR(B[i], NURBS::Bernstein(i, n, u), 1e-15) << "n = " << n << ", i = " << i << ", u = " << u;
        }
}

TEST(AllBernstein, Constexpr)
{
    static_assert(NURBS::AllBernstein(2, 0.5)[1] == 0.5);
    EXPECT_EQ(NURBS::AllBernstein(2, 0.5)[1], 0.5);
}

TEST(AllBernstein, Constraints)
{
    static_assert(AllBernsteinInvocable<double>);
    static_assert(AllBernsteinInvocable<float>);
    static_assert(!AllBernsteinInvocable<int>);
}
