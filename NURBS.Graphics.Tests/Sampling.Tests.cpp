#include <gtest/gtest.h>

#include <Graphics/Sampling.hpp>

#include <glm/glm.hpp>

#include <concepts>
#include <vector>

TEST(ParameterAt, Endpoints)
{
    EXPECT_DOUBLE_EQ(Graphics::ParameterAt(-1.0, 3.0, 0, 5), -1.0);
    EXPECT_DOUBLE_EQ(Graphics::ParameterAt(-1.0, 3.0, 4, 5), 3.0);
}

TEST(ParameterAt, Interior)
{
    EXPECT_DOUBLE_EQ(Graphics::ParameterAt(-1.0, 3.0, 2, 5), 1.0);
    EXPECT_DOUBLE_EQ(Graphics::ParameterAt(0.0, 1.0, 1, 3), 0.5);
}

TEST(ParameterAt, Descending)
{
    EXPECT_DOUBLE_EQ(Graphics::ParameterAt(2.0, 0.0, 0, 3), 2.0);
    EXPECT_DOUBLE_EQ(Graphics::ParameterAt(2.0, 0.0, 1, 3), 1.0);
    EXPECT_DOUBLE_EQ(Graphics::ParameterAt(2.0, 0.0, 2, 3), 0.0);
}

TEST(SampleCurve, Scalar)
{
    const std::vector<double> points = Graphics::SampleCurve([](double u) { return u * u; }, 0.0, 2.0, 3);
    ASSERT_EQ(points.size(), 3u);
    EXPECT_DOUBLE_EQ(points[0], 0.0);
    EXPECT_DOUBLE_EQ(points[1], 1.0);
    EXPECT_DOUBLE_EQ(points[2], 4.0);
}

TEST(SampleCurve, Vector)
{
    const auto line = [](double u) { return glm::dvec2(u, 2.0 * u); };
    const auto points = Graphics::SampleCurve(line, 1.0, 3.0, 5);
    static_assert(std::same_as<decltype(points), const std::vector<glm::dvec2>>);
    ASSERT_EQ(points.size(), 5u);
    EXPECT_EQ(points.front(), glm::dvec2(1.0, 2.0));
    EXPECT_EQ(points[2], glm::dvec2(2.0, 4.0));
    EXPECT_EQ(points.back(), glm::dvec2(3.0, 6.0));
}

TEST(SampleCurve, DeducesValueType)
{
    // A reference result is sampled by value.
    const glm::dvec3 fixed{1.0, 2.0, 3.0};
    const auto ref = [&fixed](double) -> const glm::dvec3& { return fixed; };
    const auto points = Graphics::SampleCurve(ref, 0.0, 1.0, 2);
    static_assert(std::same_as<decltype(points), const std::vector<glm::dvec3>>);
    EXPECT_EQ(points[1], fixed);
}
