#include <gtest/gtest.h>

#include <NURBS/horner1.hpp>

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

// NoScale is the opposite case: it has addition but no scalar * point. Together the two types show that the concept
// needs both operations, not just one.
struct NoScale
{
};

constexpr NoScale operator+(const NoScale& p, const NoScale&) { return p; }

template <typename R, typename S>
concept Horner1Invocable = requires(const R& a, S u) { NURBS::horner1(a, u); };

} // namespace

TEST(Horner1, Constant)
{
    const std::vector<double> a{5.0};
    EXPECT_DOUBLE_EQ(NURBS::horner1(a, 0.0), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::horner1(a, 1.0), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::horner1(a, -3.5), 5.0);
}

TEST(Horner1, Linear)
{
    const std::vector<double> a{1.0, 2.0};
    EXPECT_DOUBLE_EQ(NURBS::horner1(a, 3.0), 7.0);
}

TEST(Horner1, CubicDouble)
{
    const std::vector<double> a{1.0, 2.0, 3.0, 4.0};
    EXPECT_DOUBLE_EQ(NURBS::horner1(a, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(NURBS::horner1(a, 1.0), 10.0);
    EXPECT_DOUBLE_EQ(NURBS::horner1(a, 0.5), 3.25);
    EXPECT_DOUBLE_EQ(NURBS::horner1(a, -2.0), -23.0);
}

TEST(Horner1, CubicFloat)
{
    const std::vector<float> a{1.0f, 2.0f, 3.0f, 4.0f};
    EXPECT_FLOAT_EQ(NURBS::horner1(a, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(NURBS::horner1(a, 1.0f), 10.0f);
    EXPECT_FLOAT_EQ(NURBS::horner1(a, 0.5f), 3.25f);
    EXPECT_FLOAT_EQ(NURBS::horner1(a, -2.0f), -23.0f);
}

TEST(Horner1, AcceptsVariousContainers)
{
    const std::array<double, 4> arr{1.0, 2.0, 3.0, 4.0};
    const std::vector<double> vec{1.0, 2.0, 3.0, 4.0};
    const std::span<const double> spn{vec};
    const double carr[]{1.0, 2.0, 3.0, 4.0};

    EXPECT_DOUBLE_EQ(NURBS::horner1(arr, 0.5), 3.25);
    EXPECT_DOUBLE_EQ(NURBS::horner1(vec, 0.5), 3.25);
    EXPECT_DOUBLE_EQ(NURBS::horner1(spn, 0.5), 3.25);
    EXPECT_DOUBLE_EQ(NURBS::horner1(carr, 0.5), 3.25);
}

TEST(Horner1, Parabola2D)
{
    // C(u) = (u, u^2)
    const std::vector<glm::dvec2> a{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};

    const glm::dvec2 c1 = NURBS::horner1(a, 0.5);
    EXPECT_DOUBLE_EQ(c1.x, 0.5);
    EXPECT_DOUBLE_EQ(c1.y, 0.25);

    const glm::dvec2 c2 = NURBS::horner1(a, 2.0);
    EXPECT_DOUBLE_EQ(c2.x, 2.0);
    EXPECT_DOUBLE_EQ(c2.y, 4.0);
}

TEST(Horner1, TwistedCubic3D)
{
    // C(u) = (u, u^2, u^3)
    const std::array<glm::dvec3, 4> a{
        glm::dvec3{0.0, 0.0, 0.0}, glm::dvec3{1.0, 0.0, 0.0}, glm::dvec3{0.0, 1.0, 0.0}, glm::dvec3{0.0, 0.0, 1.0}};

    const glm::dvec3 c = NURBS::horner1(a, 2.0);
    EXPECT_DOUBLE_EQ(c.x, 2.0);
    EXPECT_DOUBLE_EQ(c.y, 4.0);
    EXPECT_DOUBLE_EQ(c.z, 8.0);
}

TEST(Horner1, TwistedCubic3DFloat)
{
    // C(u) = (u, u^2, u^3)
    const std::vector<glm::vec3> a{{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    const glm::vec3 c = NURBS::horner1(a, 0.5f);
    EXPECT_FLOAT_EQ(c.x, 0.5f);
    EXPECT_FLOAT_EQ(c.y, 0.25f);
    EXPECT_FLOAT_EQ(c.z, 0.125f);
}

TEST(Horner1, Constexpr)
{
    constexpr std::array a{1.0, 2.0, 3.0};
    static_assert(NURBS::horner1(a, 2.0) == 17.0);
    EXPECT_EQ(NURBS::horner1(a, 2.0), 17.0);
}

TEST(Horner1, Constraints)
{
    static_assert(NURBS::CurvePoint<double, double>);
    static_assert(NURBS::CurvePoint<glm::dvec2, double>);
    static_assert(NURBS::CurvePoint<glm::dvec3, double>);
    static_assert(NURBS::CurvePoint<glm::vec3, float>);
    static_assert(!NURBS::CurvePoint<NoAdd, double>);
    static_assert(!NURBS::CurvePoint<NoScale, double>);

    static_assert(Horner1Invocable<std::vector<double>, double>);
    static_assert(Horner1Invocable<std::vector<glm::dvec3>, double>);
    static_assert(!Horner1Invocable<std::vector<double>, int>);
    static_assert(!Horner1Invocable<std::vector<NoAdd>, double>);
    static_assert(!Horner1Invocable<std::vector<NoScale>, double>);
}
