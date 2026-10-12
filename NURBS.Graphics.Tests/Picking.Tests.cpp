#include <gtest/gtest.h>

#include <Graphics/OrbitCamera.hpp>
#include <Graphics/Picking.hpp>
#include <Graphics/PointDrag.hpp>

#include <glm/glm.hpp>

#include <optional>

namespace
{

void ExpectNear(glm::dvec3 actual, glm::dvec3 expected, double tolerance = 1e-9)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

// The ray through ndc for camera, as Viewport3D builds it.
Graphics::Ray RayThrough(const Graphics::OrbitCamera& camera, glm::dvec2 ndc, double aspect = 1.5)
{
    return Graphics::ScreenRay(glm::inverse(camera.Projection(aspect) * camera.View()), ndc);
}

// Where p lands on the screen, in normalized device coordinates.
glm::dvec2 ToNdc(const Graphics::OrbitCamera& camera, glm::dvec3 p, double aspect = 1.5)
{
    const glm::dvec4 clip = camera.Projection(aspect) * camera.View() * glm::dvec4(p, 1.0);
    return glm::dvec2(clip) / clip.w;
}

// The distance from p to the line along ray.
double DistanceToRay(const Graphics::Ray& ray, glm::dvec3 p)
{
    return glm::length(glm::cross(p - ray.origin, ray.direction));
}

} // namespace

TEST(ScreenRay, CenterRayRunsFromEyeTowardTarget)
{
    Graphics::OrbitCamera camera;
    camera.Fit({-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}, 1.5);
    const Graphics::Ray ray = RayThrough(camera, {0.0, 0.0});
    ExpectNear(ray.direction, glm::normalize(camera.Target() - camera.Eye()));
    EXPECT_NEAR(DistanceToRay(ray, camera.Target()), 0.0, 1e-9);
}

TEST(ScreenRay, PassesThroughProjectedPoint)
{
    for (const bool orthographic : {false, true})
    {
        Graphics::OrbitCamera camera;
        camera.SetOrthographic(orthographic);
        camera.Orbit(37.0, -12.0);
        const glm::dvec3 p{0.4, -0.7, 0.9};
        const Graphics::Ray ray = RayThrough(camera, ToNdc(camera, p));
        EXPECT_NEAR(DistanceToRay(ray, p), 0.0, 1e-9) << "orthographic " << orthographic;
        EXPECT_NEAR(glm::length(ray.direction), 1.0, 1e-12);
    }
}

TEST(IntersectPlane, HitsHorizontalPlane)
{
    const Graphics::Ray ray{{1.0, 2.0, 5.0}, glm::normalize(glm::dvec3{1.0, 0.0, -1.0})};
    const std::optional<glm::dvec3> hit = Graphics::IntersectPlane(ray, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});
    ASSERT_TRUE(hit);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): clang-tidy can't see through ASSERT_TRUE.
    ExpectNear(*hit, {5.0, 2.0, 1.0});
}

TEST(IntersectPlane, RejectsEdgeOnPlane)
{
    const Graphics::Ray ray{{0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}};
    EXPECT_FALSE(Graphics::IntersectPlane(ray, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}));
}

TEST(NearestPointOnLine, FindsClosestPointOnSkewLine)
{
    // The ray runs along x at y = 3, z = 2; the line is the z axis.
    const Graphics::Ray ray{{-4.0, 3.0, 2.0}, {1.0, 0.0, 0.0}};
    const std::optional<glm::dvec3> p = Graphics::NearestPointOnLine(ray, {0.0, 0.0, -1.0}, {0.0, 0.0, 1.0});
    ASSERT_TRUE(p);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): clang-tidy can't see through ASSERT_TRUE.
    ExpectNear(*p, {0.0, 0.0, 2.0});
}

TEST(NearestPointOnLine, RejectsParallelLine)
{
    const Graphics::Ray ray{{1.0, 0.0, 5.0}, {0.0, 0.0, -1.0}};
    EXPECT_FALSE(Graphics::NearestPointOnLine(ray, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}));
}

TEST(PointDrag, DoesNotJumpWhenAnchored)
{
    Graphics::OrbitCamera camera;
    const glm::dvec3 p{0.3, 0.2, 0.5};
    const glm::dvec3 view = glm::normalize(camera.Target() - camera.Eye());
    // Grab a little off the point, as a click within the pick radius does.
    const Graphics::Ray ray = RayThrough(camera, ToNdc(camera, p) + glm::dvec2{0.01, -0.02});

    for (const Graphics::DragConstraint c :
         {Graphics::DragConstraint::ViewPlane, Graphics::DragConstraint::Vertical, Graphics::DragConstraint::Horizontal})
    {
        Graphics::PointDrag drag;
        drag.Anchor(p, c, ray, view);
        EXPECT_EQ(drag.Constraint(), c);
        const std::optional<glm::dvec3> followed = drag.Follow(ray);
        ASSERT_TRUE(followed);
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access): clang-tidy can't see through ASSERT_TRUE.
        ExpectNear(*followed, p);
    }
}

TEST(PointDrag, ViewPlaneKeepsDepthAndFollowsMouse)
{
    Graphics::OrbitCamera camera;
    const glm::dvec3 p{0.3, 0.2, 0.5};
    const glm::dvec3 view = glm::normalize(camera.Target() - camera.Eye());
    const glm::dvec2 ndc = ToNdc(camera, p);

    Graphics::PointDrag drag;
    drag.Anchor(p, Graphics::DragConstraint::ViewPlane, RayThrough(camera, ndc), view);
    const glm::dvec2 moved = ndc + glm::dvec2{0.2, 0.1};
    const std::optional<glm::dvec3> followed = drag.Follow(RayThrough(camera, moved));
    ASSERT_TRUE(followed);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): clang-tidy can't see through ASSERT_TRUE.
    const glm::dvec3 q = *followed;
    EXPECT_NEAR(glm::dot(q - p, view), 0.0, 1e-9);
    const glm::dvec2 landed = ToNdc(camera, q);
    EXPECT_NEAR(landed.x, moved.x, 1e-9);
    EXPECT_NEAR(landed.y, moved.y, 1e-9);
}

TEST(PointDrag, VerticalMovesOnlyInZ)
{
    Graphics::OrbitCamera camera;
    const glm::dvec3 p{0.3, 0.2, 0.5};
    const glm::dvec3 view = glm::normalize(camera.Target() - camera.Eye());
    const glm::dvec2 ndc = ToNdc(camera, p);

    Graphics::PointDrag drag;
    drag.Anchor(p, Graphics::DragConstraint::Vertical, RayThrough(camera, ndc), view);
    const std::optional<glm::dvec3> followed = drag.Follow(RayThrough(camera, ndc + glm::dvec2{0.05, 0.3}));
    ASSERT_TRUE(followed);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): clang-tidy can't see through ASSERT_TRUE.
    const glm::dvec3 q = *followed;
    EXPECT_NEAR(q.x, p.x, 1e-9);
    EXPECT_NEAR(q.y, p.y, 1e-9);
    EXPECT_GT(q.z, p.z);
}

TEST(PointDrag, HorizontalKeepsHeight)
{
    Graphics::OrbitCamera camera;
    const glm::dvec3 p{0.3, 0.2, 0.5};
    const glm::dvec3 view = glm::normalize(camera.Target() - camera.Eye());
    const glm::dvec2 ndc = ToNdc(camera, p);

    Graphics::PointDrag drag;
    drag.Anchor(p, Graphics::DragConstraint::Horizontal, RayThrough(camera, ndc), view);
    const std::optional<glm::dvec3> followed = drag.Follow(RayThrough(camera, ndc + glm::dvec2{-0.2, 0.15}));
    ASSERT_TRUE(followed);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): clang-tidy can't see through ASSERT_TRUE.
    const glm::dvec3 q = *followed;
    EXPECT_NEAR(q.z, p.z, 1e-9);
    EXPECT_GT(glm::length(q - p), 0.01);
}

TEST(PointDrag, EdgeOnHorizontalFallsBackToViewPlane)
{
    // Looking along -x, level: the horizontal plane is seen edge-on.
    const Graphics::Ray ray{{5.0, 0.0, 0.5}, {-1.0, 0.0, 0.0}};
    const glm::dvec3 view{-1.0, 0.0, 0.0};
    Graphics::PointDrag drag;
    drag.Anchor({0.0, 0.0, 0.5}, Graphics::DragConstraint::Horizontal, ray, view);
    EXPECT_TRUE(drag.FellBack());
    EXPECT_EQ(drag.Constraint(), Graphics::DragConstraint::ViewPlane);
    const std::optional<glm::dvec3> q = drag.Follow(ray);
    ASSERT_TRUE(q);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): clang-tidy can't see through ASSERT_TRUE.
    ExpectNear(*q, {0.0, 0.0, 0.5});
}

TEST(PointDrag, ViewPlaneAnchorMissedKeepsNoOffset)
{
    // A ray level with the view plane misses it; the drag still anchors, with no
    // fallback and no offset, and follows once a ray hits the plane.
    const glm::dvec3 view{0.0, 0.0, -1.0};
    const Graphics::Ray level{{5.0, 0.0, 0.5}, {-1.0, 0.0, 0.0}};
    Graphics::PointDrag drag;
    drag.Anchor({0.0, 0.0, 0.5}, Graphics::DragConstraint::ViewPlane, level, view);
    EXPECT_FALSE(drag.FellBack());
    EXPECT_EQ(drag.Constraint(), Graphics::DragConstraint::ViewPlane);
    const std::optional<glm::dvec3> q = drag.Follow({{1.0, 2.0, 5.0}, view});
    ASSERT_TRUE(q);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): clang-tidy can't see through ASSERT_TRUE.
    ExpectNear(*q, {1.0, 2.0, 0.5});
}

TEST(PointDrag, FollowRejectsRayParallelToPlane)
{
    // Anchored from above, then followed along a ray level with the plane.
    const Graphics::Ray down{{0.0, 0.0, 5.0}, {0.0, 0.0, -1.0}};
    Graphics::PointDrag drag;
    drag.Anchor({0.0, 0.0, 0.5}, Graphics::DragConstraint::Horizontal, down, {0.0, 0.0, -1.0});
    ASSERT_FALSE(drag.FellBack());
    const Graphics::Ray level{{5.0, 0.0, 1.0}, {-1.0, 0.0, 0.0}};
    EXPECT_FALSE(drag.Follow(level));
}
