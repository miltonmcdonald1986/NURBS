/// @file
/// Rays through the screen and where they meet planes and lines.

#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <optional>

namespace Graphics
{

/// A ray from @p origin along the unit vector @p direction.
struct Ray
{
    /// The start of the ray.
    glm::dvec3 origin{0.0};
    /// The unit direction.
    glm::dvec3 direction{0.0, 0.0, -1.0};

    /// The point at distance @p t along the ray.
    /// @param t The distance from the origin.
    /// @return `origin + t * direction`.
    [[nodiscard]] glm::dvec3 At(double t) const { return origin + t * direction; }
};

/// The ray through a point on the screen, from the near plane toward the far
/// plane, for a perspective or orthographic projection alike.
/// @param inverseViewProjection The inverse of projection * view.
/// @param ndc The point in normalized device coordinates, each in [-1, 1], y up.
/// @return The ray.
[[nodiscard]] Ray ScreenRay(const glm::dmat4& inverseViewProjection, glm::dvec2 ndc);

/// Where @p ray meets the plane through @p point with normal @p normal.
/// @param ray The ray.
/// @param point A point on the plane.
/// @param normal The plane's unit normal.
/// @param minCosine The smallest |cos| of the angle between the ray and the
///        normal accepted; below it the plane is seen too nearly edge-on for
///        the hit to be stable.
/// @return The hit, or `std::nullopt` if the plane is too nearly edge-on.
///         The hit may lie behind the ray's origin.
[[nodiscard]] std::optional<glm::dvec3> IntersectPlane(const Ray& ray, glm::dvec3 point, glm::dvec3 normal,
                                                       double minCosine = 0.05);

/// The point on the line through @p point along @p direction nearest to @p ray.
/// @param ray The ray.
/// @param point A point on the line.
/// @param direction The line's unit direction.
/// @param minSine The smallest sine of the angle between the ray and the line
///        accepted; below it they are too nearly parallel for the result to
///        be stable.
/// @return The nearest point on the line, or `std::nullopt` if the line is
///         too nearly parallel to the ray.
[[nodiscard]] std::optional<glm::dvec3> NearestPointOnLine(const Ray& ray, glm::dvec3 point, glm::dvec3 direction,
                                                           double minSine = 0.05);

} // namespace Graphics
