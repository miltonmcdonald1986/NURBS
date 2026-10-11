#include <gtest/gtest.h>

#include <Graphics/Bounds.hpp>

#include <glm/glm.hpp>

#include <span>
#include <vector>

TEST(BoundsOf, EmptyIsNullopt)
{
    EXPECT_FALSE(Graphics::BoundsOf(std::span<const glm::dvec2>{}).has_value());
    EXPECT_FALSE(Graphics::BoundsOf(std::span<const glm::dvec3>{}).has_value());
}

TEST(BoundsOf, SinglePoint)
{
    const std::vector<glm::dvec3> points{{1.0, -2.0, 3.0}};
    const auto bounds = Graphics::BoundsOf(points);
    ASSERT_TRUE(bounds.has_value());
    EXPECT_EQ(bounds->min, points[0]);
    EXPECT_EQ(bounds->max, points[0]);
}

TEST(BoundsOf, Componentwise2D)
{
    // A std::vector converts to the span overload.
    const std::vector<glm::dvec2> points{{1.0, 5.0}, {-3.0, 2.0}, {4.0, -1.0}};
    const auto bounds = Graphics::BoundsOf(points);
    ASSERT_TRUE(bounds.has_value());
    EXPECT_EQ(bounds->min, glm::dvec2(-3.0, -1.0));
    EXPECT_EQ(bounds->max, glm::dvec2(4.0, 5.0));
}

TEST(BoundsOf, Componentwise3D)
{
    const std::vector<glm::dvec3> points{{1.0, 5.0, 0.0}, {-3.0, 2.0, 7.0}, {4.0, -1.0, -2.0}};
    const auto bounds = Graphics::BoundsOf(points);
    ASSERT_TRUE(bounds.has_value());
    EXPECT_EQ(bounds->min, glm::dvec3(-3.0, -1.0, -2.0));
    EXPECT_EQ(bounds->max, glm::dvec3(4.0, 5.0, 7.0));
}

TEST(Bounds, ExtendPoint)
{
    Graphics::Bounds2D bounds{{0.0, 0.0}, {1.0, 1.0}};
    bounds.Extend(glm::dvec2(2.0, -1.0));
    EXPECT_EQ(bounds.min, glm::dvec2(0.0, -1.0));
    EXPECT_EQ(bounds.max, glm::dvec2(2.0, 1.0));

    // A point inside changes nothing.
    bounds.Extend(glm::dvec2(0.5, 0.5));
    EXPECT_EQ(bounds.min, glm::dvec2(0.0, -1.0));
    EXPECT_EQ(bounds.max, glm::dvec2(2.0, 1.0));
}

TEST(Bounds, ExtendSpan)
{
    Graphics::Bounds3D bounds{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    const std::vector<glm::dvec3> points{{-1.0, 2.0, 0.0}, {0.0, 0.0, 3.0}};
    bounds.Extend(std::span<const glm::dvec3>(points));
    EXPECT_EQ(bounds.min, glm::dvec3(-1.0, 0.0, 0.0));
    EXPECT_EQ(bounds.max, glm::dvec3(0.0, 2.0, 3.0));
}
