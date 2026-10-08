#include <gtest/gtest.h>

#include <NURBS/bezier.hpp>

#include <glm/glm.hpp>

#include <array>
#include <span>
#include <vector>

namespace
{

// NoAdd is a dummy point type with scalar multiplication (double * NoAdd) but no operator+. It meets half of
// CurvePoint, so the concept should reject it. The operator just returns p because its body never runs; only its
// existence matters to the concept.
struct NoAdd
{
};

constexpr NoAdd operator*(double, const NoAdd& p) { return p; }

// NoScale is the opposite case: it has addition but no scalar * point.
struct NoScale
{
};

constexpr NoScale operator+(const NoScale& p, const NoScale&) { return p; }

template <typename R, typename S>
concept PointOnBezierCurveInvocable = requires(const R& P, S u) { NURBS::PointOnBezierCurve(P, u); };

template <typename R, typename S>
concept deCasteljau1Invocable = requires(const R& P, S u) { NURBS::deCasteljau1(P, u); };

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
