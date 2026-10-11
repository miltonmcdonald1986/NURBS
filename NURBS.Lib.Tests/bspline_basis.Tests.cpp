#include <gtest/gtest.h>

#include <NURBS/bernstein.hpp>
#include <NURBS/bspline_basis.hpp>

#include <array>
#include <cstddef>
#include <numeric>
#include <span>
#include <vector>

namespace
{

template <typename S, typename R>
concept FindSpanInvocable = requires(std::size_t p, S u, const R& U) { NURBS::FindSpan(p, u, U); };

template <typename S, typename R>
concept BasisFunsInvocable =
    requires(std::size_t i, S u, std::size_t p, const R& U) { NURBS::BasisFuns(i, u, p, U); };

template <typename S, typename R>
concept DersBasisFunsInvocable = requires(std::size_t i, S u, std::size_t p, std::size_t d, const R& U) {
    NURBS::DersBasisFuns(i, u, p, d, U);
};

// Example 2.3 of The NURBS Book: p = 2, n = 7.
constexpr std::size_t p = 2;
constexpr std::size_t lastSpan = 7; // n; not named n, which would hide FindSpan's local (MSVC C4459)
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
        ASSERT_LE(i, lastSpan) << "u = " << u;
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

TEST(BasisFuns, BookExample)
{
    // Example 2.3: N_{2,2}(5/2) = 1/8, N_{3,2}(5/2) = 6/8, N_{4,2}(5/2) = 1/8.
    const std::vector<double> N = NURBS::BasisFuns(4, 2.5, p, U);
    ASSERT_EQ(N.size(), p + 1);
    EXPECT_DOUBLE_EQ(N[0], 0.125);
    EXPECT_DOUBLE_EQ(N[1], 0.75);
    EXPECT_DOUBLE_EQ(N[2], 0.125);
}

TEST(BasisFuns, Start)
{
    const std::vector<double> N = NURBS::BasisFuns(2, 0.0, p, U);
    ASSERT_EQ(N.size(), p + 1);
    EXPECT_DOUBLE_EQ(N[0], 1.0);
    EXPECT_DOUBLE_EQ(N[1], 0.0);
    EXPECT_DOUBLE_EQ(N[2], 0.0);
}

TEST(BasisFuns, End)
{
    const std::vector<double> N = NURBS::BasisFuns(lastSpan, 5.0, p, U);
    ASSERT_EQ(N.size(), p + 1);
    EXPECT_DOUBLE_EQ(N[0], 0.0);
    EXPECT_DOUBLE_EQ(N[1], 0.0);
    EXPECT_DOUBLE_EQ(N[2], 1.0);
}

TEST(BasisFuns, PartitionOfUnity)
{
    for (const double u : {0.0, 0.001, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 4.999, 5.0})
    {
        const std::size_t i = NURBS::FindSpan(p, u, U);
        const std::vector<double> N = NURBS::BasisFuns(i, u, p, U);
        ASSERT_EQ(N.size(), p + 1) << "u = " << u;
        for (const double value : N)
            EXPECT_GE(value, 0.0) << "u = " << u;
        EXPECT_NEAR(std::accumulate(N.begin(), N.end(), 0.0), 1.0, 1e-12) << "u = " << u;
    }
}

TEST(BasisFuns, MatchesBernstein)
{
    // On a Bezier knot vector the B-spline basis is the Bernstein basis.
    const std::vector<double> bezier = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
    for (const double u : {0.0, 0.25, 0.5, 0.75, 1.0})
    {
        const std::vector<double> N = NURBS::BasisFuns(3, u, 3, bezier);
        const std::vector<double> B = NURBS::AllBernstein(3, u);
        ASSERT_EQ(N.size(), B.size()) << "u = " << u;
        for (std::size_t j = 0; j < N.size(); ++j)
            EXPECT_NEAR(N[j], B[j], 1e-12) << "u = " << u << ", j = " << j;
    }
}

TEST(BasisFuns, DegreeZero)
{
    const std::vector<double> step = {0.0, 1.0, 2.0, 3.0};
    const std::vector<double> N = NURBS::BasisFuns(1, 1.5, 0, step);
    ASSERT_EQ(N.size(), 1u);
    EXPECT_DOUBLE_EQ(N[0], 1.0);
}

TEST(BasisFuns, Unclamped)
{
    // The uniform quadratic B-spline at the middle of a span.
    const std::vector<double> uniform = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const std::vector<double> N = NURBS::BasisFuns(2, 2.5, 2, uniform);
    ASSERT_EQ(N.size(), 3u);
    EXPECT_DOUBLE_EQ(N[0], 0.125);
    EXPECT_DOUBLE_EQ(N[1], 0.75);
    EXPECT_DOUBLE_EQ(N[2], 0.125);
}

TEST(BasisFuns, Float)
{
    const std::vector<float> Uf = {0.0f, 0.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 4.0f, 5.0f, 5.0f, 5.0f};
    const std::vector<float> N = NURBS::BasisFuns(4, 2.5f, p, Uf);
    ASSERT_EQ(N.size(), p + 1);
    EXPECT_FLOAT_EQ(N[0], 0.125f);
    EXPECT_FLOAT_EQ(N[1], 0.75f);
    EXPECT_FLOAT_EQ(N[2], 0.125f);
}

TEST(BasisFuns, Span)
{
    const std::span<const double> view(U);
    const std::vector<double> N = NURBS::BasisFuns(4, 2.5, p, view);
    ASSERT_EQ(N.size(), p + 1);
    EXPECT_DOUBLE_EQ(N[1], 0.75);
}

TEST(BasisFuns, Constexpr)
{
    static_assert(NURBS::BasisFuns(4, 2.5, p, U)[1] == 0.75);
    static_assert(NURBS::BasisFuns(lastSpan, 5.0, p, U)[2] == 1.0);
}

TEST(BasisFuns, Constraints)
{
    static_assert(BasisFunsInvocable<double, std::vector<double>>);
    static_assert(BasisFunsInvocable<float, std::vector<float>>);
    static_assert(!BasisFunsInvocable<int, std::vector<int>>);
    static_assert(!BasisFunsInvocable<double, std::vector<float>>);
}

TEST(DersBasisFuns, BookExample)
{
    // Example 2.4: at u = 5/2 the first derivatives of N_{2,2}, N_{3,2}, N_{4,2}
    // are -1/2, 0, 1/2 and the second derivatives are 1, -2, 1.
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(4, 2.5, p, 2, U);
    ASSERT_EQ(ders.size(), 3u);
    for (const std::vector<double>& row : ders)
    {
        ASSERT_EQ(row.size(), p + 1);
    }
    EXPECT_DOUBLE_EQ(ders[0][0], 0.125);
    EXPECT_DOUBLE_EQ(ders[0][1], 0.75);
    EXPECT_DOUBLE_EQ(ders[0][2], 0.125);
    EXPECT_DOUBLE_EQ(ders[1][0], -0.5);
    EXPECT_DOUBLE_EQ(ders[1][1], 0.0);
    EXPECT_DOUBLE_EQ(ders[1][2], 0.5);
    EXPECT_DOUBLE_EQ(ders[2][0], 1.0);
    EXPECT_DOUBLE_EQ(ders[2][1], -2.0);
    EXPECT_DOUBLE_EQ(ders[2][2], 1.0);
}

TEST(DersBasisFuns, Start)
{
    // On [0, 1]: N_{0,2} = (1 - u)^2, N_{1,2} = 2u - 3u^2/2, N_{2,2} = u^2/2.
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(2, 0.0, p, 2, U);
    ASSERT_EQ(ders.size(), 3u);
    EXPECT_DOUBLE_EQ(ders[1][0], -2.0);
    EXPECT_DOUBLE_EQ(ders[1][1], 2.0);
    EXPECT_DOUBLE_EQ(ders[1][2], 0.0);
    EXPECT_DOUBLE_EQ(ders[2][0], 2.0);
    EXPECT_DOUBLE_EQ(ders[2][1], -3.0);
    EXPECT_DOUBLE_EQ(ders[2][2], 1.0);
}

TEST(DersBasisFuns, End)
{
    // On [4, 5]: N_{5,2} = (5 - u)^2, N_{6,2} = 2(u - 4)(5 - u), N_{7,2} = (u - 4)^2.
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(lastSpan, 5.0, p, 2, U);
    ASSERT_EQ(ders.size(), 3u);
    EXPECT_DOUBLE_EQ(ders[1][0], 0.0);
    EXPECT_DOUBLE_EQ(ders[1][1], -2.0);
    EXPECT_DOUBLE_EQ(ders[1][2], 2.0);
    EXPECT_DOUBLE_EQ(ders[2][0], 2.0);
    EXPECT_DOUBLE_EQ(ders[2][1], -4.0);
    EXPECT_DOUBLE_EQ(ders[2][2], 2.0);
}

TEST(DersBasisFuns, RowZeroMatchesBasisFuns)
{
    for (const double u : {0.0, 0.001, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 4.999, 5.0})
    {
        const std::size_t i = NURBS::FindSpan(p, u, U);
        const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(i, u, p, 1, U);
        const std::vector<double> N = NURBS::BasisFuns(i, u, p, U);
        ASSERT_EQ(ders[0].size(), N.size()) << "u = " << u;
        for (std::size_t j = 0; j < N.size(); ++j)
        {
            EXPECT_DOUBLE_EQ(ders[0][j], N[j]) << "u = " << u << ", j = " << j;
        }
    }
}

TEST(DersBasisFuns, DerivativesSumToZero)
{
    // The basis functions sum to 1, so each of their derivatives sums to 0.
    for (const double u : {0.0, 0.5, 1.0, 2.5, 3.5, 4.0, 4.5, 5.0})
    {
        const std::size_t i = NURBS::FindSpan(p, u, U);
        const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(i, u, p, p, U);
        for (std::size_t k = 1; k < ders.size(); ++k)
        {
            EXPECT_NEAR(std::accumulate(ders[k].begin(), ders[k].end(), 0.0), 0.0, 1e-12)
                << "u = " << u << ", k = " << k;
        }
    }
}

TEST(DersBasisFuns, MatchesFiniteDifferences)
{
    // A cubic on a non-uniform knot vector, compared with central differences
    // of BasisFuns inside each span.
    constexpr std::size_t cubic = 3;
    const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 0.7, 1.5, 1.6, 3.0, 3.0, 3.0, 3.0};
    constexpr double h = 1e-6;
    for (const double u : {0.3, 1.1, 1.55, 2.2, 2.9})
    {
        const std::size_t i = NURBS::FindSpan(cubic, u, knots);
        const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(i, u, cubic, 1, knots);
        const std::vector<double> plus = NURBS::BasisFuns(i, u + h, cubic, knots);
        const std::vector<double> minus = NURBS::BasisFuns(i, u - h, cubic, knots);
        for (std::size_t j = 0; j <= cubic; ++j)
        {
            EXPECT_NEAR(ders[1][j], (plus[j] - minus[j]) / (2 * h), 1e-6) << "u = " << u << ", j = " << j;
        }
    }
}

TEST(DersBasisFuns, MatchesBernsteinDerivative)
{
    // On a Bezier knot vector, B'_{j,3} = 3 (B_{j-1,2} - B_{j,2}).
    const std::vector<double> bezier = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
    for (const double u : {0.0, 0.25, 0.5, 0.75, 1.0})
    {
        const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(3, u, 3, 1, bezier);
        const std::vector<double> B = NURBS::AllBernstein(2, u);
        for (std::size_t j = 0; j <= 3; ++j)
        {
            const double lower = j > 0 ? B[j - 1] : 0.0;
            const double same = j < 3 ? B[j] : 0.0;
            EXPECT_NEAR(ders[1][j], 3.0 * (lower - same), 1e-12) << "u = " << u << ", j = " << j;
        }
    }
}

TEST(DersBasisFuns, HigherThanDegreeIsZero)
{
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(4, 2.5, p, p + 2, U);
    ASSERT_EQ(ders.size(), p + 3);
    for (std::size_t k = p + 1; k < ders.size(); ++k)
    {
        ASSERT_EQ(ders[k].size(), p + 1);
        for (const double value : ders[k])
        {
            EXPECT_EQ(value, 0.0) << "k = " << k;
        }
    }
}

TEST(DersBasisFuns, ZeroDerivatives)
{
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(4, 2.5, p, 0, U);
    ASSERT_EQ(ders.size(), 1u);
    EXPECT_EQ(ders[0], NURBS::BasisFuns(4, 2.5, p, U));
}

TEST(DersBasisFuns, DegreeZero)
{
    const std::vector<double> step = {0.0, 1.0, 2.0, 3.0};
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(1, 1.5, 0, 1, step);
    ASSERT_EQ(ders.size(), 2u);
    ASSERT_EQ(ders[0].size(), 1u);
    ASSERT_EQ(ders[1].size(), 1u);
    EXPECT_DOUBLE_EQ(ders[0][0], 1.0);
    EXPECT_DOUBLE_EQ(ders[1][0], 0.0);
}

TEST(DersBasisFuns, Unclamped)
{
    // The uniform quadratic B-spline at the middle of a span, as in BasisFuns.Unclamped.
    const std::vector<double> uniform = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(2, 2.5, 2, 2, uniform);
    EXPECT_DOUBLE_EQ(ders[1][0], -0.5);
    EXPECT_DOUBLE_EQ(ders[1][1], 0.0);
    EXPECT_DOUBLE_EQ(ders[1][2], 0.5);
    EXPECT_DOUBLE_EQ(ders[2][0], 1.0);
    EXPECT_DOUBLE_EQ(ders[2][1], -2.0);
    EXPECT_DOUBLE_EQ(ders[2][2], 1.0);
}

TEST(DersBasisFuns, Float)
{
    const std::vector<float> Uf = {0.0f, 0.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 4.0f, 5.0f, 5.0f, 5.0f};
    const std::vector<std::vector<float>> ders = NURBS::DersBasisFuns(4, 2.5f, p, 2, Uf);
    ASSERT_EQ(ders.size(), 3u);
    EXPECT_FLOAT_EQ(ders[1][0], -0.5f);
    EXPECT_FLOAT_EQ(ders[1][2], 0.5f);
    EXPECT_FLOAT_EQ(ders[2][1], -2.0f);
}

TEST(DersBasisFuns, Span)
{
    const std::span<const double> view(U);
    const std::vector<std::vector<double>> ders = NURBS::DersBasisFuns(4, 2.5, p, 1, view);
    ASSERT_EQ(ders.size(), 2u);
    EXPECT_DOUBLE_EQ(ders[1][2], 0.5);
}

TEST(DersBasisFuns, Constexpr)
{
    static_assert(NURBS::DersBasisFuns(4, 2.5, p, 2, U)[1][0] == -0.5);
    static_assert(NURBS::DersBasisFuns(4, 2.5, p, 2, U)[2][1] == -2.0);
    static_assert(NURBS::DersBasisFuns(lastSpan, 5.0, p, 1, U)[1][2] == 2.0);
}

TEST(DersBasisFuns, Constraints)
{
    static_assert(DersBasisFunsInvocable<double, std::vector<double>>);
    static_assert(DersBasisFunsInvocable<float, std::vector<float>>);
    static_assert(!DersBasisFunsInvocable<int, std::vector<int>>);
    static_assert(!DersBasisFunsInvocable<double, std::vector<float>>);
}
