#include <gtest/gtest.h>

#include <NURBS/bspline_basis.hpp>

#include <array>
#include <cstddef>
#include <span>
#include <vector>

namespace
{

template <typename S, typename R>
concept FindSpanInvocable = requires(std::size_t p, S u, const R& U) { NURBS::FindSpan(p, u, U); };

// Example 2.3 of The NURBS Book: p = 2, n = 7.
constexpr std::size_t p = 2;
constexpr std::size_t n = 7;
constexpr std::array<double, 11> U = {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 5.0, 5.0, 5.0};

} // namespace

TEST(FindSpan, BookExample)
{
    EXPECT_EQ(NURBS::FindSpan(p, 2.5, U), 4u);
}

TEST(FindSpan, Start)
{
    EXPECT_EQ(NURBS::FindSpan(p, 0.0, U), 2u);
}

TEST(FindSpan, End)
{
    EXPECT_EQ(NURBS::FindSpan(p, 5.0, U), 7u);
}

TEST(FindSpan, InteriorKnots)
{
    EXPECT_EQ(NURBS::FindSpan(p, 1.0, U), 3u);
    EXPECT_EQ(NURBS::FindSpan(p, 2.0, U), 4u);
    EXPECT_EQ(NURBS::FindSpan(p, 3.0, U), 5u);
    // u = 4 is a double knot, so span 6 is empty and u falls in span 7.
    EXPECT_EQ(NURBS::FindSpan(p, 4.0, U), 7u);
}

TEST(FindSpan, EveryNonEmptySpan)
{
    for (const double u : {0.5, 1.5, 2.5, 3.5, 4.5, 0.001, 4.999})
    {
        const std::size_t i = NURBS::FindSpan(p, u, U);
        ASSERT_GE(i, p) << "u = " << u;
        ASSERT_LE(i, n) << "u = " << u;
        EXPECT_LE(U[i], u) << "u = " << u;
        EXPECT_LT(u, U[i + 1]) << "u = " << u;
    }
}

TEST(FindSpan, SingleSpan)
{
    // A Bezier knot vector has one non-empty span.
    const std::vector<double> bezier = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
    EXPECT_EQ(NURBS::FindSpan(3, 0.0, bezier), 3u);
    EXPECT_EQ(NURBS::FindSpan(3, 0.5, bezier), 3u);
    EXPECT_EQ(NURBS::FindSpan(3, 1.0, bezier), 3u);
}

TEST(FindSpan, Unclamped)
{
    // A uniform knot vector with p = 2 and n = 3. The spans 0, 1, 4 and 5 are
    // non-empty but lie outside the domain [u_2, u_4] = [2, 4].
    const std::vector<double> uniform = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    EXPECT_EQ(NURBS::FindSpan(2, 2.0, uniform), 2u);
    EXPECT_EQ(NURBS::FindSpan(2, 2.5, uniform), 2u);
    EXPECT_EQ(NURBS::FindSpan(2, 3.0, uniform), 3u);
    EXPECT_EQ(NURBS::FindSpan(2, 3.5, uniform), 3u);
    EXPECT_EQ(NURBS::FindSpan(2, 4.0, uniform), 3u);
}

TEST(FindSpan, Float)
{
    const std::vector<float> Uf = {0.0f, 0.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 4.0f, 5.0f, 5.0f, 5.0f};
    EXPECT_EQ(NURBS::FindSpan(p, 2.5f, Uf), 4u);
    EXPECT_EQ(NURBS::FindSpan(p, 4.0f, Uf), 7u);
    EXPECT_EQ(NURBS::FindSpan(p, 5.0f, Uf), 7u);
}

TEST(FindSpan, Span)
{
    const std::span<const double> view(U);
    EXPECT_EQ(NURBS::FindSpan(p, 2.5, view), 4u);
}

TEST(FindSpan, Constexpr)
{
    static_assert(NURBS::FindSpan(p, 2.5, U) == 4);
    static_assert(NURBS::FindSpan(p, 5.0, U) == 7);
}

TEST(FindSpan, Constraints)
{
    static_assert(FindSpanInvocable<double, std::vector<double>>);
    static_assert(FindSpanInvocable<float, std::vector<float>>);
    static_assert(!FindSpanInvocable<int, std::vector<int>>);
    static_assert(!FindSpanInvocable<double, std::vector<float>>);
}
