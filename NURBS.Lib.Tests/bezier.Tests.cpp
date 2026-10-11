#include "TestPointTypes.hpp"

#include <gtest/gtest.h>

#include <NURBS/bezier.hpp>

#include <glm/glm.hpp>

#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <span>
#include <utility>
#include <vector>

namespace
{

using NURBS::Tests::NoAdd;
using NURBS::Tests::NoScale;

template <typename R, typename S>
concept PointOnBezierCurveInvocable = requires(const R& P, S u) { NURBS::PointOnBezierCurve(P, u); };

template <typename R, typename S>
concept deCasteljau1Invocable = requires(const R& P, S u) { NURBS::deCasteljau1(P, u); };

template <typename R, typename S>
concept deCasteljau2Invocable = requires(const R& P, S u, S v) { NURBS::deCasteljau2(P, u, v); };

// Evaluates the double sum directly, as sum B_{i,n}(u) B_{j,m}(v) P[i][j].
template <typename Point, std::size_t N, std::size_t M>
Point BernsteinSum(const std::array<std::array<Point, M>, N>& P, double u, double v)
{
    Point S{};
    for (std::size_t i = 0; i < N; ++i)
    {
        for (std::size_t j = 0; j < M; ++j)
        {
            S += NURBS::Bernstein(i, N - 1, u) * NURBS::Bernstein(j, M - 1, v) * P[i][j];
        }
    }
    return S;
}

template <typename P, typename... S>
concept BezierCurveInstantiable = requires { typename NURBS::BezierCurve<P, S...>; };

template <typename P, typename... S>
concept RationalBezierCurveInstantiable = requires { typename NURBS::RationalBezierCurve<P, S...>; };

} // namespace

TEST(PointOnBezierCurve, Degree0)
{
    const std::vector<double> P{5.0};
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(P, 0.0), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(P, 0.3), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(P, 1.0), 5.0);
}

TEST(PointOnBezierCurve, Linear)
{
    // C(u) = (1-u) * P0 + u * P1
    const std::vector<double> P{1.0, 3.0};
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(P, 0.25), 1.5);
}

TEST(PointOnBezierCurve, CubicDouble)
{
    const std::vector<double> P{1.0, 2.0, 4.0, 8.0};
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(P, 0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(P, 0.25), 125.0 / 64.0);
}

TEST(PointOnBezierCurve, CubicFloat)
{
    const std::vector<float> P{1.0f, 2.0f, 4.0f, 8.0f};
    EXPECT_FLOAT_EQ(NURBS::PointOnBezierCurve(P, 0.5f), 27.0f / 8.0f);
    EXPECT_FLOAT_EQ(NURBS::PointOnBezierCurve(P, 0.25f), 125.0f / 64.0f);
}

TEST(PointOnBezierCurve, Endpoints)
{
    // A Bezier curve interpolates its first and last control points.
    const std::vector<glm::dvec2> P{{1.0, 2.0}, {3.0, 5.0}, {-1.0, 4.0}, {6.0, -2.0}};

    const glm::dvec2 c0 = NURBS::PointOnBezierCurve(P, 0.0);
    EXPECT_DOUBLE_EQ(c0.x, P.front().x);
    EXPECT_DOUBLE_EQ(c0.y, P.front().y);

    const glm::dvec2 c1 = NURBS::PointOnBezierCurve(P, 1.0);
    EXPECT_DOUBLE_EQ(c1.x, P.back().x);
    EXPECT_DOUBLE_EQ(c1.y, P.back().y);
}

TEST(PointOnBezierCurve, Parabola2D)
{
    // C(u) = (u, u^2)
    const std::vector<glm::dvec2> P{{0.0, 0.0}, {0.5, 0.0}, {1.0, 1.0}};

    const glm::dvec2 c1 = NURBS::PointOnBezierCurve(P, 0.5);
    EXPECT_DOUBLE_EQ(c1.x, 0.5);
    EXPECT_DOUBLE_EQ(c1.y, 0.25);

    const glm::dvec2 c2 = NURBS::PointOnBezierCurve(P, 0.3);
    EXPECT_DOUBLE_EQ(c2.x, 0.3);
    EXPECT_DOUBLE_EQ(c2.y, 0.09);
}

TEST(PointOnBezierCurve, TwistedCubic3D)
{
    // C(u) = (u, u^2, u^3)
    const std::array<glm::dvec3, 4> P{glm::dvec3{0.0, 0.0, 0.0},
                                      glm::dvec3{1.0 / 3.0, 0.0, 0.0},
                                      glm::dvec3{2.0 / 3.0, 1.0 / 3.0, 0.0},
                                      glm::dvec3{1.0, 1.0, 1.0}};

    const glm::dvec3 c = NURBS::PointOnBezierCurve(P, 0.5);
    EXPECT_NEAR(c.x, 0.5, 1e-15);
    EXPECT_NEAR(c.y, 0.25, 1e-15);
    EXPECT_NEAR(c.z, 0.125, 1e-15);
}

TEST(PointOnBezierCurve, TwistedCubic3DFloat)
{
    // C(u) = (u, u^2, u^3)
    const std::vector<glm::vec3> P{
        {0.0f, 0.0f, 0.0f}, {1.0f / 3.0f, 0.0f, 0.0f}, {2.0f / 3.0f, 1.0f / 3.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};

    const glm::vec3 c = NURBS::PointOnBezierCurve(P, 0.5f);
    EXPECT_FLOAT_EQ(c.x, 0.5f);
    EXPECT_FLOAT_EQ(c.y, 0.25f);
    EXPECT_FLOAT_EQ(c.z, 0.125f);
}

TEST(PointOnBezierCurve, AcceptsVariousContainers)
{
    const std::array<double, 4> arr{1.0, 2.0, 4.0, 8.0};
    const std::vector<double> vec{1.0, 2.0, 4.0, 8.0};
    const std::span<const double> spn{vec};
    const double carr[]{1.0, 2.0, 4.0, 8.0};

    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(arr, 0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(vec, 0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(spn, 0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::PointOnBezierCurve(carr, 0.5), 27.0 / 8.0);
}

TEST(PointOnBezierCurve, Constexpr)
{
    constexpr std::array P{1.0, 2.0, 4.0};
    static_assert(NURBS::PointOnBezierCurve(P, 0.5) == 2.25);
    EXPECT_EQ(NURBS::PointOnBezierCurve(P, 0.5), 2.25);
}

TEST(PointOnBezierCurve, Constraints)
{
    static_assert(PointOnBezierCurveInvocable<std::vector<double>, double>);
    static_assert(PointOnBezierCurveInvocable<std::vector<glm::dvec3>, double>);
    static_assert(PointOnBezierCurveInvocable<std::vector<glm::vec3>, float>);
    static_assert(!PointOnBezierCurveInvocable<std::vector<double>, int>);
    static_assert(!PointOnBezierCurveInvocable<std::vector<NoAdd>, double>);
    static_assert(!PointOnBezierCurveInvocable<std::vector<NoScale>, double>);
}

TEST(deCasteljau1, Degree0)
{
    const std::vector<double> P{5.0};
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(P, 0.0), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(P, 0.3), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(P, 1.0), 5.0);
}

TEST(deCasteljau1, Linear)
{
    // C(u) = (1-u) * P0 + u * P1
    const std::vector<double> P{1.0, 3.0};
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(P, 0.25), 1.5);
}

TEST(deCasteljau1, CubicDouble)
{
    const std::vector<double> P{1.0, 2.0, 4.0, 8.0};
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(P, 0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(P, 0.25), 125.0 / 64.0);
}

TEST(deCasteljau1, CubicFloat)
{
    const std::vector<float> P{1.0f, 2.0f, 4.0f, 8.0f};
    EXPECT_FLOAT_EQ(NURBS::deCasteljau1(P, 0.5f), 27.0f / 8.0f);
    EXPECT_FLOAT_EQ(NURBS::deCasteljau1(P, 0.25f), 125.0f / 64.0f);
}

TEST(deCasteljau1, Endpoints)
{
    // A Bezier curve interpolates its first and last control points.
    const std::vector<glm::dvec2> P{{1.0, 2.0}, {3.0, 5.0}, {-1.0, 4.0}, {6.0, -2.0}};

    const glm::dvec2 c0 = NURBS::deCasteljau1(P, 0.0);
    EXPECT_DOUBLE_EQ(c0.x, P.front().x);
    EXPECT_DOUBLE_EQ(c0.y, P.front().y);

    const glm::dvec2 c1 = NURBS::deCasteljau1(P, 1.0);
    EXPECT_DOUBLE_EQ(c1.x, P.back().x);
    EXPECT_DOUBLE_EQ(c1.y, P.back().y);
}

TEST(deCasteljau1, Parabola2D)
{
    // C(u) = (u, u^2)
    const std::vector<glm::dvec2> P{{0.0, 0.0}, {0.5, 0.0}, {1.0, 1.0}};

    const glm::dvec2 c1 = NURBS::deCasteljau1(P, 0.5);
    EXPECT_DOUBLE_EQ(c1.x, 0.5);
    EXPECT_DOUBLE_EQ(c1.y, 0.25);

    const glm::dvec2 c2 = NURBS::deCasteljau1(P, 0.3);
    EXPECT_DOUBLE_EQ(c2.x, 0.3);
    EXPECT_DOUBLE_EQ(c2.y, 0.09);
}

TEST(deCasteljau1, TwistedCubic3D)
{
    // C(u) = (u, u^2, u^3)
    const std::array<glm::dvec3, 4> P{glm::dvec3{0.0, 0.0, 0.0},
                                      glm::dvec3{1.0 / 3.0, 0.0, 0.0},
                                      glm::dvec3{2.0 / 3.0, 1.0 / 3.0, 0.0},
                                      glm::dvec3{1.0, 1.0, 1.0}};

    const glm::dvec3 c = NURBS::deCasteljau1(P, 0.5);
    EXPECT_NEAR(c.x, 0.5, 1e-15);
    EXPECT_NEAR(c.y, 0.25, 1e-15);
    EXPECT_NEAR(c.z, 0.125, 1e-15);
}

TEST(deCasteljau1, TwistedCubic3DFloat)
{
    // C(u) = (u, u^2, u^3)
    const std::vector<glm::vec3> P{
        {0.0f, 0.0f, 0.0f}, {1.0f / 3.0f, 0.0f, 0.0f}, {2.0f / 3.0f, 1.0f / 3.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};

    const glm::vec3 c = NURBS::deCasteljau1(P, 0.5f);
    EXPECT_FLOAT_EQ(c.x, 0.5f);
    EXPECT_FLOAT_EQ(c.y, 0.25f);
    EXPECT_FLOAT_EQ(c.z, 0.125f);
}

TEST(deCasteljau1, AcceptsVariousContainers)
{
    const std::array<double, 4> arr{1.0, 2.0, 4.0, 8.0};
    const std::vector<double> vec{1.0, 2.0, 4.0, 8.0};
    const std::span<const double> spn{vec};
    const double carr[]{1.0, 2.0, 4.0, 8.0};

    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(arr, 0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(vec, 0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(spn, 0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau1(carr, 0.5), 27.0 / 8.0);
}

TEST(deCasteljau1, Constexpr)
{
    constexpr std::array P{1.0, 2.0, 4.0};
    static_assert(NURBS::deCasteljau1(P, 0.5) == 2.25);
    EXPECT_EQ(NURBS::deCasteljau1(P, 0.5), 2.25);
}

TEST(deCasteljau1, Constraints)
{
    static_assert(deCasteljau1Invocable<std::vector<double>, double>);
    static_assert(deCasteljau1Invocable<std::vector<glm::dvec3>, double>);
    static_assert(deCasteljau1Invocable<std::vector<glm::vec3>, float>);
    static_assert(!deCasteljau1Invocable<std::vector<double>, int>);
    static_assert(!deCasteljau1Invocable<std::vector<NoAdd>, double>);
    static_assert(!deCasteljau1Invocable<std::vector<NoScale>, double>);
}

TEST(deCasteljau1, MatchesPointOnBezierCurve)
{
    const std::vector<glm::dvec3> P{{1.0, 2.0, 3.0}, {-2.0, 4.0, 0.5}, {3.0, -1.0, 2.0}, {0.0, 5.0, -4.0}};

    for (const double u : {0.0, 0.1, 0.37, 0.5, 0.9, 1.0})
    {
        const glm::dvec3 expected = NURBS::PointOnBezierCurve(P, u);
        const glm::dvec3 actual = NURBS::deCasteljau1(P, u);
        EXPECT_NEAR(actual.x, expected.x, 1e-14);
        EXPECT_NEAR(actual.y, expected.y, 1e-14);
        EXPECT_NEAR(actual.z, expected.z, 1e-14);
    }
}

TEST(deCasteljau2, Degree0)
{
    const std::vector<std::vector<double>> P{{5.0}};
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(P, 0.0, 0.0), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(P, 0.3, 0.8), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(P, 1.0, 1.0), 5.0);
}

TEST(deCasteljau2, Bilinear)
{
    // S(u,v) = (1-u)(1-v) P00 + (1-u) v P01 + u (1-v) P10 + u v P11
    const std::vector<std::vector<double>> P{{1.0, 2.0}, {3.0, 5.0}};
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(P, 0.5, 0.5), 2.75);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(P, 0.25, 0.75), 0.1875 * 1.0 + 0.5625 * 2.0 + 0.0625 * 3.0 + 0.1875 * 5.0);
}

TEST(deCasteljau2, Corners)
{
    // A Bezier surface interpolates its four corner control points.
    const std::vector<std::vector<glm::dvec3>> P{
        {{0.0, 0.0, 1.0}, {1.0, 0.0, 2.0}, {2.0, 0.0, -1.0}},
        {{0.0, 1.0, 3.0}, {1.0, 1.0, 4.0}, {2.0, 1.0, 0.5}},
    };

    const auto expectEq = [](const glm::dvec3& actual, const glm::dvec3& expected) {
        EXPECT_DOUBLE_EQ(actual.x, expected.x);
        EXPECT_DOUBLE_EQ(actual.y, expected.y);
        EXPECT_DOUBLE_EQ(actual.z, expected.z);
    };
    expectEq(NURBS::deCasteljau2(P, 0.0, 0.0), P[0][0]);
    expectEq(NURBS::deCasteljau2(P, 0.0, 1.0), P[0][2]);
    expectEq(NURBS::deCasteljau2(P, 1.0, 0.0), P[1][0]);
    expectEq(NURBS::deCasteljau2(P, 1.0, 1.0), P[1][2]);
}

TEST(deCasteljau2, SingleRowIsCurveInV)
{
    const std::vector<double> row{1.0, 2.0, 4.0, 8.0};
    const std::vector<std::vector<double>> P{row};
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(P, 0.7, 0.25), NURBS::deCasteljau1(row, 0.25));
}

TEST(deCasteljau2, SingleColumnIsCurveInU)
{
    const std::vector<double> column{1.0, 2.0, 4.0, 8.0};
    const std::vector<std::vector<double>> P{{1.0}, {2.0}, {4.0}, {8.0}};
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(P, 0.25, 0.7), NURBS::deCasteljau1(column, 0.25));
}

TEST(deCasteljau2, MatchesBernsteinSum)
{
    // Degree 1 in u and 3 in v evaluates columns first; the transpose evaluates rows first.
    const std::array<std::array<glm::dvec3, 4>, 2> wide{{
        {glm::dvec3{1.0, 2.0, 3.0}, glm::dvec3{-2.0, 4.0, 0.5}, glm::dvec3{3.0, -1.0, 2.0}, glm::dvec3{0.0, 5.0, -4.0}},
        {glm::dvec3{2.0, 0.0, 1.0}, glm::dvec3{1.0, 3.0, -2.0}, glm::dvec3{-1.0, 2.0, 4.0}, glm::dvec3{5.0, 1.0, 0.0}},
    }};
    std::array<std::array<glm::dvec3, 2>, 4> tall{};
    for (std::size_t i = 0; i < 2; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            tall[j][i] = wide[i][j];
        }
    }

    for (const double u : {0.0, 0.1, 0.37, 0.5, 1.0})
    {
        for (const double v : {0.0, 0.25, 0.6, 0.9, 1.0})
        {
            const glm::dvec3 expectedWide = BernsteinSum(wide, u, v);
            const glm::dvec3 actualWide = NURBS::deCasteljau2(wide, u, v);
            EXPECT_NEAR(actualWide.x, expectedWide.x, 1e-14);
            EXPECT_NEAR(actualWide.y, expectedWide.y, 1e-14);
            EXPECT_NEAR(actualWide.z, expectedWide.z, 1e-14);

            const glm::dvec3 expectedTall = BernsteinSum(tall, v, u);
            const glm::dvec3 actualTall = NURBS::deCasteljau2(tall, v, u);
            EXPECT_NEAR(actualTall.x, expectedTall.x, 1e-14);
            EXPECT_NEAR(actualTall.y, expectedTall.y, 1e-14);
            EXPECT_NEAR(actualTall.z, expectedTall.z, 1e-14);

            // Transposing the grid and swapping u and v gives the same surface.
            EXPECT_NEAR(actualTall.x, actualWide.x, 1e-14);
            EXPECT_NEAR(actualTall.y, actualWide.y, 1e-14);
            EXPECT_NEAR(actualTall.z, actualWide.z, 1e-14);
        }
    }
}

TEST(deCasteljau2, HyperbolicParaboloid3DFloat)
{
    // S(u,v) = (u, v, uv)
    const std::vector<std::vector<glm::vec3>> P{
        {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},
        {{1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}},
    };

    const glm::vec3 S = NURBS::deCasteljau2(P, 0.5f, 0.25f);
    EXPECT_FLOAT_EQ(S.x, 0.5f);
    EXPECT_FLOAT_EQ(S.y, 0.25f);
    EXPECT_FLOAT_EQ(S.z, 0.125f);
}

TEST(deCasteljau2, AcceptsVariousContainers)
{
    // S(u,v) = (1-u)(1-v) + 2(1-u)v + 3u(1-v) + 5uv, so S(0.5, 0.5) = 2.75.
    const std::array<std::array<double, 2>, 2> arr{{{1.0, 2.0}, {3.0, 5.0}}};
    const std::vector<std::vector<double>> vec{{1.0, 2.0}, {3.0, 5.0}};
    const std::vector<double> row0{1.0, 2.0};
    const std::vector<double> row1{3.0, 5.0};
    const std::vector<std::span<const double>> spn{row0, row1};
    const double carr[2][2]{{1.0, 2.0}, {3.0, 5.0}};

    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(arr, 0.5, 0.5), 2.75);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(vec, 0.5, 0.5), 2.75);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(spn, 0.5, 0.5), 2.75);
    EXPECT_DOUBLE_EQ(NURBS::deCasteljau2(carr, 0.5, 0.5), 2.75);
}

TEST(deCasteljau2, Constexpr)
{
    // One grid per branch: degree (1, 2) evaluates columns first and degree (2, 1) rows first.
    constexpr std::array<std::array<double, 3>, 2> wide{{{1.0, 2.0, 4.0}, {1.0, 2.0, 4.0}}};
    constexpr std::array<std::array<double, 2>, 3> tall{{{1.0, 1.0}, {2.0, 2.0}, {4.0, 4.0}}};
    static_assert(NURBS::deCasteljau2(wide, 0.25, 0.5) == 2.25);
    static_assert(NURBS::deCasteljau2(tall, 0.5, 0.25) == 2.25);
    EXPECT_EQ(NURBS::deCasteljau2(wide, 0.25, 0.5), 2.25);
    EXPECT_EQ(NURBS::deCasteljau2(tall, 0.5, 0.25), 2.25);
}

TEST(deCasteljau2, Constraints)
{
    static_assert(deCasteljau2Invocable<std::vector<std::vector<double>>, double>);
    static_assert(deCasteljau2Invocable<std::vector<std::vector<glm::dvec3>>, double>);
    static_assert(deCasteljau2Invocable<std::array<std::array<glm::vec3, 3>, 2>, float>);
    static_assert(!deCasteljau2Invocable<std::vector<std::vector<double>>, int>);
    static_assert(!deCasteljau2Invocable<std::vector<std::vector<NoAdd>>, double>);
    static_assert(!deCasteljau2Invocable<std::vector<std::vector<NoScale>>, double>);
    // Rows must be ranges.
    static_assert(!deCasteljau2Invocable<std::vector<double>, double>);
}

TEST(BezierCurve, Degree0)
{
    const NURBS::BezierCurve curve{std::vector<double>{5.0}};
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.0), 5.0);
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.3), 5.0);
    EXPECT_DOUBLE_EQ(curve.Evaluate(1.0), 5.0);
}

TEST(BezierCurve, CubicDouble)
{
    const NURBS::BezierCurve curve{std::vector<double>{1.0, 2.0, 4.0, 8.0}};
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.25), 125.0 / 64.0);
}

TEST(BezierCurve, CubicFloat)
{
    const NURBS::BezierCurve curve{std::vector<float>{1.0f, 2.0f, 4.0f, 8.0f}};
    EXPECT_FLOAT_EQ(curve.Evaluate(0.5f), 27.0f / 8.0f);
    EXPECT_FLOAT_EQ(curve.Evaluate(0.25f), 125.0f / 64.0f);
}

TEST(BezierCurve, Endpoints)
{
    // A Bezier curve interpolates its first and last control points.
    const std::vector<glm::dvec2> P{{1.0, 2.0}, {3.0, 5.0}, {-1.0, 4.0}, {6.0, -2.0}};
    const NURBS::BezierCurve curve{P};

    const glm::dvec2 c0 = curve.Evaluate(0.0);
    EXPECT_DOUBLE_EQ(c0.x, P.front().x);
    EXPECT_DOUBLE_EQ(c0.y, P.front().y);

    const glm::dvec2 c1 = curve.Evaluate(1.0);
    EXPECT_DOUBLE_EQ(c1.x, P.back().x);
    EXPECT_DOUBLE_EQ(c1.y, P.back().y);
}

TEST(BezierCurve, Parabola2D)
{
    // C(u) = (u, u^2)
    const NURBS::BezierCurve<glm::dvec2> curve{{0.0, 0.0}, {0.5, 0.0}, {1.0, 1.0}};

    const glm::dvec2 c1 = curve.Evaluate(0.5);
    EXPECT_DOUBLE_EQ(c1.x, 0.5);
    EXPECT_DOUBLE_EQ(c1.y, 0.25);

    const glm::dvec2 c2 = curve.Evaluate(0.3);
    EXPECT_DOUBLE_EQ(c2.x, 0.3);
    EXPECT_DOUBLE_EQ(c2.y, 0.09);
}

TEST(BezierCurve, TwistedCubic3DFloat)
{
    // C(u) = (u, u^2, u^3)
    const NURBS::BezierCurve curve{glm::vec3{0.0f, 0.0f, 0.0f}, glm::vec3{1.0f / 3.0f, 0.0f, 0.0f},
                                   glm::vec3{2.0f / 3.0f, 1.0f / 3.0f, 0.0f}, glm::vec3{1.0f, 1.0f, 1.0f}};

    const glm::vec3 c = curve.Evaluate(0.5f);
    EXPECT_FLOAT_EQ(c.x, 0.5f);
    EXPECT_FLOAT_EQ(c.y, 0.25f);
    EXPECT_FLOAT_EQ(c.z, 0.125f);
}

TEST(BezierCurve, AcceptsVariousContainers)
{
    const std::array<double, 4> arr{1.0, 2.0, 4.0, 8.0};
    const std::vector<double> vec{1.0, 2.0, 4.0, 8.0};
    const std::span<const double> spn{vec};
    const double carr[]{1.0, 2.0, 4.0, 8.0};

    EXPECT_DOUBLE_EQ(NURBS::BezierCurve{arr}.Evaluate(0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::BezierCurve{vec}.Evaluate(0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::BezierCurve{spn}.Evaluate(0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(NURBS::BezierCurve{carr}.Evaluate(0.5), 27.0 / 8.0);
}

TEST(BezierCurve, DeducesTypes)
{
    static_assert(std::same_as<decltype(NURBS::BezierCurve{std::declval<const std::vector<double>&>()}),
                               NURBS::BezierCurve<double, double>>);
    static_assert(std::same_as<decltype(NURBS::BezierCurve{std::declval<const std::vector<glm::dvec3>&>()}),
                               NURBS::BezierCurve<glm::dvec3, double>>);
    static_assert(std::same_as<decltype(NURBS::BezierCurve{std::declval<const std::vector<glm::vec3>&>()}),
                               NURBS::BezierCurve<glm::vec3, float>>);

    // Braced lists of points deduce from the initializer_list constructor.
    static_assert(std::same_as<decltype(NURBS::BezierCurve{1.0, 2.0}), NURBS::BezierCurve<double, double>>);
    static_assert(std::same_as<decltype(NURBS::BezierCurve{1.0f, 2.0f}), NURBS::BezierCurve<float, float>>);
    static_assert(std::same_as<decltype(NURBS::BezierCurve{glm::dvec3{}, glm::dvec3{}}),
                               NURBS::BezierCurve<glm::dvec3, double>>);

    // A single braced container must still deduce from the range constructor, not as a one-point
    // initializer_list whose point type is the container.
    static_assert(std::same_as<decltype(NURBS::BezierCurve{std::declval<const std::array<double, 4>&>()}),
                               NURBS::BezierCurve<double, double>>);
    static_assert(std::same_as<decltype(NURBS::BezierCurve{std::declval<const std::span<const double>&>()}),
                               NURBS::BezierCurve<double, double>>);
    static_assert(std::same_as<decltype(NURBS::BezierCurve{std::declval<const double (&)[4]>()}),
                               NURBS::BezierCurve<double, double>>);
}

TEST(BezierCurve, InitializerList)
{
    const NURBS::BezierCurve curve{1.0, 2.0, 4.0, 8.0};
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.5), 27.0 / 8.0);
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.25), 125.0 / 64.0);

    const NURBS::BezierCurve degree0{5.0};
    EXPECT_DOUBLE_EQ(degree0.Evaluate(0.3), 5.0);
}

TEST(BezierCurve, Constexpr)
{
    constexpr auto evaluate = [] {
        constexpr std::array P{1.0, 2.0, 4.0};
        return NURBS::BezierCurve{P}.Evaluate(0.5);
    };
    static_assert(evaluate() == 2.25);
    EXPECT_EQ(evaluate(), 2.25);
}

TEST(BezierCurve, Constraints)
{
    static_assert(BezierCurveInstantiable<double>);
    static_assert(BezierCurveInstantiable<glm::dvec3>);
    static_assert(BezierCurveInstantiable<glm::vec3>);
    static_assert(!BezierCurveInstantiable<int>);
    static_assert(!BezierCurveInstantiable<glm::ivec3>);
    static_assert(!BezierCurveInstantiable<NoAdd, double>);
    static_assert(!BezierCurveInstantiable<NoScale, double>);
}

TEST(BezierCurve, MatchesPointOnBezierCurve)
{
    const std::vector<glm::dvec3> P{{1.0, 2.0, 3.0}, {-2.0, 4.0, 0.5}, {3.0, -1.0, 2.0}, {0.0, 5.0, -4.0}};
    const NURBS::BezierCurve curve{P};

    for (const double u : {0.0, 0.1, 0.37, 0.5, 0.9, 1.0})
    {
        const glm::dvec3 expected = NURBS::PointOnBezierCurve(P, u);
        const glm::dvec3 actual = curve.Evaluate(u);
        EXPECT_NEAR(actual.x, expected.x, 1e-14);
        EXPECT_NEAR(actual.y, expected.y, 1e-14);
        EXPECT_NEAR(actual.z, expected.z, 1e-14);
    }
}

TEST(BezierCurve, Degree)
{
    EXPECT_EQ(NURBS::BezierCurve{5.0}.Degree(), 0u);
    EXPECT_EQ((NURBS::BezierCurve{1.0, 2.0, 4.0, 8.0}.Degree()), 3u);
}

TEST(BezierCurve, ControlPoints)
{
    const std::vector<glm::dvec2> P{{0.0, 0.0}, {0.5, 1.0}, {1.0, 0.0}};
    const NURBS::BezierCurve curve{P};

    const std::span<const glm::dvec2> points = curve.ControlPoints();
    ASSERT_EQ(points.size(), P.size());
    for (std::size_t i = 0; i < P.size(); ++i)
    {
        EXPECT_EQ(points[i], P[i]);
    }
}

TEST(BezierCurve, EditControlPoints)
{
    // Moving the last control point moves the end of the curve with it.
    NURBS::BezierCurve curve{1.0, 2.0, 4.0};
    curve.ControlPoints()[2] = 8.0;

    EXPECT_DOUBLE_EQ(curve.Evaluate(1.0), 8.0);
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.5), 3.25);
}

TEST(RationalBezierCurve, UnitWeightsMatchBezierCurve)
{
    // With every weight 1 the denominator is sum B_{k,n}(u) = 1, so the curve is polynomial.
    const std::vector<glm::dvec3> P{{1.0, 2.0, 3.0}, {-2.0, 4.0, 0.5}, {3.0, -1.0, 2.0}, {0.0, 5.0, -4.0}};
    const NURBS::BezierCurve bezier{P};
    const NURBS::RationalBezierCurve rational{P};
    const NURBS::RationalBezierCurve explicitOnes{P, std::vector<double>(P.size(), 1.0)};

    for (const double u : {0.0, 0.1, 0.37, 0.5, 0.9, 1.0})
    {
        const glm::dvec3 expected = bezier.Evaluate(u);
        const glm::dvec3 actual = rational.Evaluate(u);
        const glm::dvec3 actualOnes = explicitOnes.Evaluate(u);
        EXPECT_NEAR(actual.x, expected.x, 1e-14);
        EXPECT_NEAR(actual.y, expected.y, 1e-14);
        EXPECT_NEAR(actual.z, expected.z, 1e-14);
        EXPECT_NEAR(actualOnes.x, expected.x, 1e-14);
        EXPECT_NEAR(actualOnes.y, expected.y, 1e-14);
        EXPECT_NEAR(actualOnes.z, expected.z, 1e-14);
    }
}

TEST(RationalBezierCurve, Endpoints)
{
    // A rational Bezier curve interpolates its first and last control points for any positive weights.
    const std::vector<glm::dvec2> P{{1.0, 2.0}, {3.0, 5.0}, {-1.0, 4.0}, {6.0, -2.0}};
    const std::vector<double> w{0.5, 3.0, 0.25, 2.0};
    const NURBS::RationalBezierCurve curve{P, w};

    const glm::dvec2 c0 = curve.Evaluate(0.0);
    EXPECT_DOUBLE_EQ(c0.x, P.front().x);
    EXPECT_DOUBLE_EQ(c0.y, P.front().y);

    const glm::dvec2 c1 = curve.Evaluate(1.0);
    EXPECT_DOUBLE_EQ(c1.x, P.back().x);
    EXPECT_DOUBLE_EQ(c1.y, P.back().y);
}

TEST(RationalBezierCurve, QuarterCircle)
{
    // The quadratic with weights (1, sqrt(2)/2, 1) is exactly the unit quarter circle,
    // which no polynomial Bezier curve can represent.
    const std::vector<glm::dvec2> P{{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    const std::vector<double> w{1.0, std::sqrt(2.0) / 2.0, 1.0};
    const NURBS::RationalBezierCurve curve{P, w};

    for (const double u : {0.0, 0.1, 0.25, 0.37, 0.5, 0.75, 0.9, 1.0})
    {
        EXPECT_NEAR(glm::length(curve.Evaluate(u)), 1.0, 1e-14);
    }

    // By symmetry the midpoint lies on the diagonal.
    const glm::dvec2 mid = curve.Evaluate(0.5);
    EXPECT_NEAR(mid.x, std::sqrt(2.0) / 2.0, 1e-14);
    EXPECT_NEAR(mid.y, std::sqrt(2.0) / 2.0, 1e-14);
}

TEST(RationalBezierCurve, WeightScaleInvariance)
{
    // Scaling every weight by the same factor scales numerator and denominator alike.
    const std::vector<glm::dvec2> P{{0.0, 0.0}, {1.0, 2.0}, {3.0, -1.0}, {4.0, 1.0}};
    const std::vector<double> w{1.0, 2.0, 0.5, 1.5};
    std::vector<double> scaled = w;
    for (double& wi : scaled)
    {
        wi *= 7.0;
    }

    const NURBS::RationalBezierCurve curve{P, w};
    const NURBS::RationalBezierCurve scaledCurve{P, scaled};

    for (const double u : {0.0, 0.2, 0.5, 0.8, 1.0})
    {
        const glm::dvec2 a = curve.Evaluate(u);
        const glm::dvec2 b = scaledCurve.Evaluate(u);
        EXPECT_NEAR(a.x, b.x, 1e-14);
        EXPECT_NEAR(a.y, b.y, 1e-14);
    }
}

TEST(RationalBezierCurve, CubicFloat)
{
    // Parabola C(u) = (u, u^2) with unit weights, in float.
    const std::vector<glm::vec2> P{{0.0f, 0.0f}, {0.5f, 0.0f}, {1.0f, 1.0f}};
    const NURBS::RationalBezierCurve curve{P};

    const glm::vec2 c = curve.Evaluate(0.5f);
    EXPECT_FLOAT_EQ(c.x, 0.5f);
    EXPECT_FLOAT_EQ(c.y, 0.25f);
}

TEST(RationalBezierCurve, Constexpr)
{
    // C(u) = (P0 (1-u)^2 + 2 w1 P1 u (1-u) + P2 u^2) / ((1-u)^2 + 2 w1 u (1-u) + u^2) with
    // P = 0, 1, 0 and w1 = 3: at u = 0.5 this is 1.5 / 2 = 0.75.
    constexpr auto evaluate = [] {
        constexpr std::array P{0.0, 1.0, 0.0};
        constexpr std::array w{1.0, 3.0, 1.0};
        return NURBS::RationalBezierCurve{P, w}.Evaluate(0.5);
    };
    constexpr double value = evaluate();
    EXPECT_DOUBLE_EQ(value, 0.75);
}

TEST(RationalBezierCurve, DeducesTypes)
{
    static_assert(std::same_as<decltype(NURBS::RationalBezierCurve{std::declval<const std::vector<double>&>()}),
                               NURBS::RationalBezierCurve<double, double>>);
    static_assert(std::same_as<decltype(NURBS::RationalBezierCurve{std::declval<const std::vector<glm::dvec3>&>(),
                                                                   std::declval<const std::vector<double>&>()}),
                               NURBS::RationalBezierCurve<glm::dvec3, double>>);
    static_assert(std::same_as<decltype(NURBS::RationalBezierCurve{std::declval<const std::array<glm::vec3, 3>&>(),
                                                                   std::declval<const std::array<float, 3>&>()}),
                               NURBS::RationalBezierCurve<glm::vec3, float>>);
}

TEST(RationalBezierCurve, Constraints)
{
    static_assert(RationalBezierCurveInstantiable<double>);
    static_assert(RationalBezierCurveInstantiable<glm::dvec3>);
    static_assert(RationalBezierCurveInstantiable<glm::vec3>);
    static_assert(!RationalBezierCurveInstantiable<int>);
    static_assert(!RationalBezierCurveInstantiable<glm::ivec3>);
    static_assert(!RationalBezierCurveInstantiable<NoAdd, double>);
    static_assert(!RationalBezierCurveInstantiable<NoScale, double>);
}

TEST(RationalBezierCurve, Degree)
{
    EXPECT_EQ(NURBS::RationalBezierCurve{std::vector<double>{5.0}}.Degree(), 0u);
    EXPECT_EQ((NURBS::RationalBezierCurve{std::vector<double>{1.0, 2.0, 4.0, 8.0}}.Degree()), 3u);
}

TEST(RationalBezierCurve, ControlPointsAndWeights)
{
    const std::vector<glm::dvec2> P{{0.0, 0.0}, {0.5, 1.0}, {1.0, 0.0}};
    const std::vector<double> w{1.0, 2.0, 0.5};
    const NURBS::RationalBezierCurve curve{P, w};

    const std::span<const glm::dvec2> points = curve.ControlPoints();
    const std::span<const double> weights = curve.Weights();
    ASSERT_EQ(points.size(), P.size());
    ASSERT_EQ(weights.size(), w.size());
    for (std::size_t i = 0; i < P.size(); ++i)
    {
        EXPECT_EQ(points[i], P[i]);
        EXPECT_EQ(weights[i], w[i]);
    }
}

TEST(RationalBezierCurve, EditControlPoints)
{
    // Moving the last control point moves the end of the curve with it.
    NURBS::RationalBezierCurve curve{std::vector<double>{1.0, 2.0, 4.0}, std::vector<double>{1.0, 2.0, 1.0}};
    curve.ControlPoints()[2] = 8.0;

    EXPECT_DOUBLE_EQ(curve.Evaluate(1.0), 8.0);
    // (0.25 * 1 + 0.5 * 2 * 2 + 0.25 * 8) / (0.25 + 0.5 * 2 + 0.25)
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.5), 17.0 / 6.0);
}

TEST(RationalBezierCurve, EditWeights)
{
    // Raising the middle weight pulls the curve toward the middle control point.
    NURBS::RationalBezierCurve curve{std::vector<double>{0.0, 1.0, 0.0}};
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.5), 0.5);

    curve.Weights()[1] = 3.0;
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.5), 0.75);

    curve.Weights()[1] = 0.5;
    EXPECT_DOUBLE_EQ(curve.Evaluate(0.5), 1.0 / 3.0);
}
