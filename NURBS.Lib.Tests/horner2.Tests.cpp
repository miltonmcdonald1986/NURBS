#include <gtest/gtest.h>

#include <NURBS/horner1.hpp>
#include <NURBS/horner2.hpp>

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <span>
#include <vector>

namespace
{

template <typename R, typename S>
concept Horner2Invocable = requires(const R& a, S u, S v) { NURBS::horner2(a, u, v); };

// Evaluates the double sum directly, as [1 u u^2 ...] * A * [1 v v^2 ...]^T.
template <std::size_t N, std::size_t M>
double BruteForce(const std::array<std::array<double, M>, N>& a, double u, double v)
{
    double S = 0.0;
    double ui = 1.0;
    for (std::size_t i = 0; i < N; ++i)
    {
        double vj = 1.0;
        for (std::size_t j = 0; j < M; ++j)
        {
            S += a[i][j] * ui * vj;
            vj *= v;
        }
        ui *= u;
    }
    return S;
}

} // namespace

TEST(Horner2, Constant)
{
    const std::vector<std::vector<double>> a{{5.0}};
    EXPECT_DOUBLE_EQ(NURBS::horner2(a, 0.0, 0.0), 5.0);
    EXPECT_DOUBLE_EQ(NURBS::horner2(a, 1.0, -2.0), 5.0);
}

TEST(Horner2, Bilinear)
{
    // S(u,v) = 1 + 2v + 3u + 4uv
    const std::vector<std::vector<double>> a{{1.0, 2.0}, {3.0, 4.0}};
    EXPECT_DOUBLE_EQ(NURBS::horner2(a, 2.0, 3.0), 37.0);
    EXPECT_DOUBLE_EQ(NURBS::horner2(a, 0.0, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(NURBS::horner2(a, 1.0, 0.0), 4.0);
    EXPECT_DOUBLE_EQ(NURBS::horner2(a, 0.0, 1.0), 3.0);
}

TEST(Horner2, NonSquareMatchesBruteForce)
{
    // Degree 1 in u, degree 2 in v.
    const std::array<std::array<double, 3>, 2> a{{{1.0, -2.0, 0.5}, {3.0, 4.0, -1.5}}};
    for (const double u : {-1.0, 0.0, 0.25, 2.0})
    {
        for (const double v : {-0.5, 0.0, 0.75, 3.0})
        {
            EXPECT_DOUBLE_EQ(NURBS::horner2(a, u, v), BruteForce(a, u, v));
        }
    }
}

TEST(Horner2, BicubicMatchesMatrixForm)
{
    // A scalar bicubic patch is u^T A v for a 4x4 matrix A.
    const std::array<std::array<double, 4>, 4> a{{
        {1.0, 2.0, -1.0, 0.5},
        {0.0, 3.0, 1.0, -2.0},
        {4.0, -1.0, 0.0, 1.0},
        {-0.5, 1.0, 2.0, 3.0},
    }};
    for (const double u : {-1.0, 0.0, 0.5, 1.5})
    {
        for (const double v : {-2.0, 0.0, 0.3, 1.0})
        {
            EXPECT_DOUBLE_EQ(NURBS::horner2(a, u, v), BruteForce(a, u, v));
        }
    }
}

TEST(Horner2, SingleRowIsCurveInV)
{
    const std::vector<double> row{1.0, 2.0, 3.0, 4.0};
    const std::vector<std::vector<double>> a{row};
    EXPECT_DOUBLE_EQ(NURBS::horner2(a, 7.0, 0.5), NURBS::horner1(row, 0.5));
}

TEST(Horner2, SingleColumnIsCurveInU)
{
    const std::vector<double> column{1.0, 2.0, 3.0, 4.0};
    const std::vector<std::vector<double>> a{{1.0}, {2.0}, {3.0}, {4.0}};
    EXPECT_DOUBLE_EQ(NURBS::horner2(a, 0.5, 7.0), NURBS::horner1(column, 0.5));
}

TEST(Horner2, Paraboloid3D)
{
    // S(u,v) = (u, v, u^2 + v^2)
    const glm::dvec3 zero{0.0, 0.0, 0.0};
    const std::vector<std::vector<glm::dvec3>> a{
        {zero, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
        {{1.0, 0.0, 0.0}, zero, zero},
        {{0.0, 0.0, 1.0}, zero, zero},
    };

    const glm::dvec3 S = NURBS::horner2(a, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(S.x, 2.0);
    EXPECT_DOUBLE_EQ(S.y, 3.0);
    EXPECT_DOUBLE_EQ(S.z, 13.0);
}

TEST(Horner2, HyperbolicParaboloid3DFloat)
{
    // S(u,v) = (u, v, uv)
    const std::vector<std::vector<glm::vec3>> a{
        {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},
        {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
    };

    const glm::vec3 S = NURBS::horner2(a, 0.5f, -4.0f);
    EXPECT_FLOAT_EQ(S.x, 0.5f);
    EXPECT_FLOAT_EQ(S.y, -4.0f);
    EXPECT_FLOAT_EQ(S.z, -2.0f);
}

TEST(Horner2, AcceptsVariousContainers)
{
    // S(u,v) = 1 + 2v + 3u + 4uv, so S(2,3) = 37.
    const std::array<std::array<double, 2>, 2> arr{{{1.0, 2.0}, {3.0, 4.0}}};
    const std::vector<std::vector<double>> vec{{1.0, 2.0}, {3.0, 4.0}};
    const std::vector<double> row0{1.0, 2.0};
    const std::vector<double> row1{3.0, 4.0};
    const std::vector<std::span<const double>> spn{row0, row1};
    const double carr[2][2]{{1.0, 2.0}, {3.0, 4.0}};

    EXPECT_DOUBLE_EQ(NURBS::horner2(arr, 2.0, 3.0), 37.0);
    EXPECT_DOUBLE_EQ(NURBS::horner2(vec, 2.0, 3.0), 37.0);
    EXPECT_DOUBLE_EQ(NURBS::horner2(spn, 2.0, 3.0), 37.0);
    EXPECT_DOUBLE_EQ(NURBS::horner2(carr, 2.0, 3.0), 37.0);
}

TEST(Horner2, Constexpr)
{
    constexpr std::array<std::array<double, 2>, 2> a{{{1.0, 2.0}, {3.0, 4.0}}};
    static_assert(NURBS::horner2(a, 2.0, 3.0) == 37.0);
    EXPECT_EQ(NURBS::horner2(a, 2.0, 3.0), 37.0);
}

TEST(Horner2, Constraints)
{
    static_assert(Horner2Invocable<std::vector<std::vector<double>>, double>);
    static_assert(Horner2Invocable<std::vector<std::vector<glm::dvec3>>, double>);
    static_assert(Horner2Invocable<std::array<std::array<glm::vec3, 3>, 2>, float>);
    static_assert(!Horner2Invocable<std::vector<std::vector<double>>, int>);
    // Rows must be ranges.
    static_assert(!Horner2Invocable<std::vector<double>, double>);
    // glm matrices are not ranges; copy them into arrays first.
    static_assert(!Horner2Invocable<glm::dmat4, double>);
}
