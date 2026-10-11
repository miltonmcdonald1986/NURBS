#include <gtest/gtest.h>

#include <Graphics/OrbitCamera.hpp>
#include <Graphics/Theme.hpp>

#include <glm/glm.hpp>

#include <cmath>
#include <numbers>

namespace
{

void ExpectNear(glm::dvec3 actual, glm::dvec3 expected, double tolerance = 1e-12)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

} // namespace

TEST(OrbitCamera, EyeIsAtDistance)
{
    const Graphics::OrbitCamera camera;
    EXPECT_NEAR(glm::length(camera.Eye() - camera.Target()), camera.Distance(), 1e-12);
    // The default pitch is above the xy plane.
    EXPECT_GT(camera.Eye().z, camera.Target().z);
}

TEST(OrbitCamera, ViewLooksDownNegativeZ)
{
    Graphics::OrbitCamera camera;
    camera.Fit({1.0, 2.0, 3.0}, {3.0, 4.0, 5.0}, 1.0);
    const glm::dvec4 target = camera.View() * glm::dvec4(camera.Target(), 1.0);
    ExpectNear(glm::dvec3(target), {0.0, 0.0, -camera.Distance()});
    const glm::dvec4 eye = camera.View() * glm::dvec4(camera.Eye(), 1.0);
    ExpectNear(glm::dvec3(eye), {0.0, 0.0, 0.0});
}

TEST(OrbitCamera, OrbitKeepsDistanceAndTarget)
{
    Graphics::OrbitCamera camera;
    const glm::dvec3 eye = camera.Eye();
    camera.Orbit(40.0, -25.0);
    EXPECT_NEAR(glm::length(camera.Eye() - camera.Target()), camera.Distance(), 1e-12);
    EXPECT_EQ(camera.Target(), glm::dvec3(0.0, 0.0, 0.0));
    EXPECT_GT(glm::length(camera.Eye() - eye), 0.1);
}

TEST(OrbitCamera, OrbitClampsPitch)
{
    Graphics::OrbitCamera camera;
    camera.Orbit(0.0, 1e6);
    const double height = camera.Eye().z - camera.Target().z;
    EXPECT_LT(height, camera.Distance());
    EXPECT_GT(height, 0.99 * camera.Distance());

    camera.Orbit(0.0, -1e6);
    const double depth = camera.Target().z - camera.Eye().z;
    EXPECT_LT(depth, camera.Distance());
    EXPECT_GT(depth, 0.99 * camera.Distance());

    // Short of the poles, the view matrix stays finite.
    const glm::dmat4 view = camera.View();
    for (int c = 0; c < 4; ++c)
    {
        for (int r = 0; r < 4; ++r)
        {
            EXPECT_TRUE(std::isfinite(view[c][r]));
        }
    }
}

TEST(OrbitCamera, Zoom)
{
    Graphics::OrbitCamera camera;
    const double distance = camera.Distance();
    camera.Zoom(1.0);
    EXPECT_NEAR(camera.Distance(), distance / Graphics::Theme::kZoomPerWheelStep, 1e-12);
    camera.Zoom(-1.0);
    EXPECT_NEAR(camera.Distance(), distance, 1e-12);
}

TEST(OrbitCamera, ZoomClamps)
{
    Graphics::OrbitCamera camera;
    camera.Zoom(1e6);
    EXPECT_GT(camera.Distance(), 0.0);
    camera.Zoom(-1e6);
    EXPECT_TRUE(std::isfinite(camera.Distance()));
}

TEST(OrbitCamera, PanMovesInViewPlane)
{
    Graphics::OrbitCamera camera;
    const glm::dvec3 offset = camera.Eye() - camera.Target();
    const glm::dvec3 forward = -glm::normalize(offset);

    // A full viewport height of drag moves the target by the view's height at the target's depth.
    constexpr double kHeight = 600.0;
    camera.Pan(0.0, kHeight, kHeight);
    const glm::dvec3 moved = camera.Target();
    EXPECT_NEAR(glm::dot(moved, forward), 0.0, 1e-12);
    EXPECT_NEAR(glm::length(moved), 2.0 * camera.Distance() * std::tan(std::numbers::pi / 8.0), 1e-12);

    // The eye moves along with the target.
    ExpectNear(camera.Eye() - camera.Target(), offset);

    camera.Pan(50.0, 0.0, kHeight);
    EXPECT_NEAR(glm::dot(camera.Target() - moved, forward), 0.0, 1e-12);
    EXPECT_NEAR(camera.Target().z, moved.z, 1e-12);
}

TEST(OrbitCamera, Fit)
{
    Graphics::OrbitCamera camera;
    camera.Fit({-1.0, -1.0, -1.0}, {3.0, 1.0, 1.0}, 1.0);
    ExpectNear(camera.Target(), {1.0, 0.0, 0.0});
    // With aspect 1 the half field of view is 22.5 degrees.
    const double radius = 0.5 * std::sqrt(24.0);
    EXPECT_NEAR(camera.Distance(), radius / (Graphics::Theme::kFitMargin * std::sin(std::numbers::pi / 8.0)), 1e-12);
}

TEST(OrbitCamera, FitBiggerBoxBacksOff)
{
    Graphics::OrbitCamera small;
    small.Fit({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, 1.5);
    Graphics::OrbitCamera big;
    big.Fit({0.0, 0.0, 0.0}, {10.0, 10.0, 10.0}, 1.5);
    EXPECT_GT(big.Distance(), small.Distance());

    // A narrow viewport needs more distance than a wide one.
    Graphics::OrbitCamera narrow;
    narrow.Fit({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, 0.5);
    EXPECT_GT(narrow.Distance(), small.Distance());
}

TEST(OrbitCamera, FitDegenerateBox)
{
    Graphics::OrbitCamera camera;
    camera.Fit({2.0, 2.0, 2.0}, {2.0, 2.0, 2.0}, 1.0);
    ExpectNear(camera.Target(), {2.0, 2.0, 2.0});
    EXPECT_GT(camera.Distance(), 0.0);
}

TEST(OrbitCamera, Projection)
{
    Graphics::OrbitCamera camera;
    EXPECT_FALSE(camera.IsOrthographic());
    const glm::dmat4 perspective = camera.Projection(2.0);
    EXPECT_DOUBLE_EQ(perspective[2][3], -1.0);
    EXPECT_DOUBLE_EQ(perspective[3][3], 0.0);
    EXPECT_NEAR(perspective[0][0] * 2.0, perspective[1][1], 1e-12);

    camera.SetOrthographic(true);
    EXPECT_TRUE(camera.IsOrthographic());
    const glm::dmat4 orthographic = camera.Projection(2.0);
    EXPECT_DOUBLE_EQ(orthographic[2][3], 0.0);
    EXPECT_DOUBLE_EQ(orthographic[3][3], 1.0);
    EXPECT_NEAR(orthographic[0][0] * 2.0, orthographic[1][1], 1e-12);

    // At the target's depth both show the same height.
    const double halfHeight = camera.Distance() * std::tan(std::numbers::pi / 8.0);
    EXPECT_NEAR(orthographic[1][1], 1.0 / halfHeight, 1e-12);
    EXPECT_NEAR(perspective[1][1], camera.Distance() / halfHeight, 1e-12);
}
