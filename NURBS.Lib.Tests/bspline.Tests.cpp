#include "TestPointTypes.hpp"

#include <gtest/gtest.h>

#include <NURBS/bezier.hpp>
#include <NURBS/bspline.hpp>
#include <NURBS/bspline_basis.hpp>

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <span>
#include <vector>

namespace
{

using NURBS::Tests::NoAdd;
using NURBS::Tests::NoScale;

template <typename RU, typename RP, typename S>
concept CurvePointInvocable = requires(std::size_t p, const RU& U, const RP& P, S u) { NURBS::CurvePoint(p, U, P, u); };

// Example 3.1 of The NURBS Book: p = 2, n = 7.
constexpr std::size_t p = 2;
constexpr std::array<double, 11> U = {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 5.0, 5.0, 5.0};

const std::vector<glm::dvec2> P{{0.0, 0.0}, {1.0, 2.0},  {3.0, 3.0}, {4.0, 1.0},
                                {6.0, 0.0}, {7.0, -2.0}, {9.0, 1.0}, {10.0, 3.0}};

// Evaluates sum N_{i,p}(u) P[i] over every basis function, with OneBasisFun.
template <typename Point>
Point BasisSum(std::size_t deg, const std::vector<double>& knots, const std::vector<Point>& points, double u)
{
    Point C{};
    for (std::size_t i = 0; i < points.size(); ++i)
        C += NURBS::OneBasisFun(i, u, deg, knots) * points[i];
    return C;
}

} // namespace

TEST(CurvePoint, BookExample)
{
    // Example 3.1: C(5/2) = 1/8 P2 + 6/8 P3 + 1/8 P4.
    const glm::dvec2 C = NURBS::CurvePoint(p, U, P, 2.5);
    const glm::dvec2 expected = 0.125 * P[2] + 0.75 * P[3] + 0.125 * P[4];
    EXPECT_DOUBLE_EQ(C.x, expected.x);
    EXPECT_DOUBLE_EQ(C.y, expected.y);
}

TEST(CurvePoint, Endpoints)
{
    // A curve on a clamped knot vector interpolates its first and last control points.
    const glm::dvec2 c0 = NURBS::CurvePoint(p, U, P, 0.0);
    EXPECT_DOUBLE_EQ(c0.x, P.front().x);
    EXPECT_DOUBLE_EQ(c0.y, P.front().y);

    const glm::dvec2 c1 = NURBS::CurvePoint(p, U, P, 5.0);
    EXPECT_DOUBLE_EQ(c1.x, P.back().x);
    EXPECT_DOUBLE_EQ(c1.y, P.back().y);
}

TEST(CurvePoint, DoubleKnot)
{
    // u = 4 is a double knot, so a degree 2 curve passes through P5 there.
    const glm::dvec2 C = NURBS::CurvePoint(p, U, P, 4.0);
    EXPECT_DOUBLE_EQ(C.x, P[5].x);
    EXPECT_DOUBLE_EQ(C.y, P[5].y);
}

TEST(CurvePoint, MatchesBasisSum)
{
    const std::vector<double> knots(U.begin(), U.end());
    for (const double u : {0.0, 0.001, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 4.999, 5.0})
    {
        const glm::dvec2 C = NURBS::CurvePoint(p, knots, P, u);
        const glm::dvec2 expected = BasisSum(p, knots, P, u);
        EXPECT_NEAR(C.x, expected.x, 1e-12) << "u = " << u;
        EXPECT_NEAR(C.y, expected.y, 1e-12) << "u = " << u;
    }
}

TEST(CurvePoint, Unclamped)
{
    // A uniform knot vector with p = 2 and n = 3; the domain is [u_2, u_4] = [2, 4].
    const std::vector<double> uniform = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const std::vector<double> Q{1.0, 4.0, -2.0, 3.0};
    for (const double u : {2.0, 2.25, 2.5, 3.0, 3.75, 4.0})
        EXPECT_NEAR(NURBS::CurvePoint(2, uniform, Q, u), BasisSum(2, uniform, Q, u), 1e-12) << "u = " << u;

    // A uniform quadratic starts at the midpoint of its first two control points.
    EXPECT_DOUBLE_EQ(NURBS::CurvePoint(2, uniform, Q, 2.0), 2.5);
}

TEST(CurvePoint, LinearPrecision)
{
    // With the Greville abscissae xi_i = (u_{i+1} + ... + u_{i+p}) / p as
    // control points, the curve is C(u) = u.
    for (const std::size_t deg : {1u, 2u, 3u})
    {
        const std::vector<double> knots = [deg] {
            std::vector<double> k(deg + 1, 0.0);
            for (const double interior : {0.5, 1.0, 1.0, 2.5, 3.0})
                k.push_back(interior);
            k.insert(k.end(), deg + 1, 4.0);
            return k;
        }();
        const std::size_t count = knots.size() - deg - 1;
        std::vector<double> greville(count, 0.0);
        for (std::size_t i = 0; i < count; ++i)
        {
            for (std::size_t j = 1; j <= deg; ++j)
            {
                greville[i] += knots[i + j];
            }
            greville[i] /= static_cast<double>(deg);
        }
        for (const double u : {0.0, 0.25, 0.5, 1.0, 1.75, 2.5, 3.2, 4.0})
            EXPECT_NEAR(NURBS::CurvePoint(deg, knots, greville, u), u, 1e-12) << "p = " << deg << ", u = " << u;
    }
}

TEST(CurvePoint, MatchesBezier)
{
    // On a Bezier knot vector a B-spline curve is a Bezier curve.
    const std::vector<double> bezier = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
    const std::array<glm::dvec3, 4> Q{glm::dvec3{0.0, 0.0, 0.0}, glm::dvec3{1.0, 2.0, -1.0}, glm::dvec3{3.0, -1.0, 2.0},
                                      glm::dvec3{4.0, 1.0, 0.5}};
    for (const double u : {0.0, 0.25, 0.5, 0.75, 1.0})
    {
        const glm::dvec3 C = NURBS::CurvePoint(3, bezier, Q, u);
        const glm::dvec3 expected = NURBS::deCasteljau1(Q, u);
        EXPECT_NEAR(C.x, expected.x, 1e-12) << "u = " << u;
        EXPECT_NEAR(C.y, expected.y, 1e-12) << "u = " << u;
        EXPECT_NEAR(C.z, expected.z, 1e-12) << "u = " << u;
    }
}

TEST(CurvePoint, Degree0)
{
    // A degree 0 curve is piecewise constant, taking the value P[i] on span i.
    const std::vector<double> knots = {0.0, 1.0, 2.0, 3.0};
    const std::vector<double> Q{5.0, -1.0, 7.0};
    EXPECT_DOUBLE_EQ(NURBS::CurvePoint(0, knots, Q, 0.5), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::CurvePoint(0, knots, Q, 1.0), -1.0);
    EXPECT_DOUBLE_EQ(NURBS::CurvePoint(0, knots, Q, 2.5), 7.0);
    EXPECT_DOUBLE_EQ(NURBS::CurvePoint(0, knots, Q, 3.0), 7.0);
}

TEST(CurvePoint, Float)
{
    const std::vector<float> Uf = {0.0f, 0.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 4.0f, 5.0f, 5.0f, 5.0f};
    const std::vector<glm::vec2> Pf(P.begin(), P.end());
    const glm::vec2 C = NURBS::CurvePoint(p, Uf, Pf, 2.5f);
    const glm::vec2 expected = 0.125f * Pf[2] + 0.75f * Pf[3] + 0.125f * Pf[4];
    EXPECT_FLOAT_EQ(C.x, expected.x);
    EXPECT_FLOAT_EQ(C.y, expected.y);
}

TEST(CurvePoint, Span)
{
    const std::span<const double> knots(U);
    const std::span<const glm::dvec2> points(P);
    const glm::dvec2 C = NURBS::CurvePoint(p, knots, points, 2.5);
    const glm::dvec2 expected = 0.125 * P[2] + 0.75 * P[3] + 0.125 * P[4];
    EXPECT_DOUBLE_EQ(C.x, expected.x);
    EXPECT_DOUBLE_EQ(C.y, expected.y);
}

TEST(CurvePoint, Constexpr)
{
    // C(5/2) = 1/8 * 16 + 6/8 * 24 + 1/8 * 32 = 24.
    constexpr std::array<double, 8> Q = {0.0, 8.0, 16.0, 24.0, 32.0, 40.0, 48.0, 56.0};
    static_assert(NURBS::CurvePoint(p, U, Q, 2.5) == 24.0);
    static_assert(NURBS::CurvePoint(p, U, Q, 0.0) == 0.0);
    static_assert(NURBS::CurvePoint(p, U, Q, 5.0) == 56.0);
}

TEST(CurvePoint, Constraints)
{
    static_assert(CurvePointInvocable<std::vector<double>, std::vector<double>, double>);
    static_assert(CurvePointInvocable<std::vector<double>, std::vector<glm::dvec3>, double>);
    static_assert(CurvePointInvocable<std::vector<float>, std::vector<glm::vec3>, float>);
    static_assert(!CurvePointInvocable<std::vector<int>, std::vector<double>, int>);
    static_assert(!CurvePointInvocable<std::vector<float>, std::vector<double>, double>);
    static_assert(!CurvePointInvocable<std::vector<double>, std::vector<NoAdd>, double>);
    static_assert(!CurvePointInvocable<std::vector<double>, std::vector<NoScale>, double>);
}
