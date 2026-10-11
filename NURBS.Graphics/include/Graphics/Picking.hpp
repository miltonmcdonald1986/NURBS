/// @file
/// Rays through the screen and the geometry for dragging points in 3D.

#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
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

/// How a dragged point may move.
enum class DragConstraint : std::uint8_t
{
    /// In the plane through the point facing the camera.
    ViewPlane,
    /// Along the vertical (z) line through the point.
    Vertical,
    /// In the horizontal (xy) plane through the point.
    Horizontal,
};

/// One point being dragged in 3D, following a mouse ray under a constraint.
///
/// Anchor when the drag starts or its constraint changes, then call Follow
/// each frame. The point keeps its offset from the constraint's hit at the
/// anchor, so it never jumps under the cursor.
class PointDrag
{
public:
    /// Starts following from @p p under @p constraint.
    /// @param p The point's position.
    /// @param constraint How it may move.
    /// @param ray The mouse ray now.
    /// @param viewDirection The camera's unit view direction, for DragConstraint::ViewPlane.
    void Anchor(glm::dvec3 p, DragConstraint constraint, const Ray& ray, glm::dvec3 viewDirection);

    /// The point's new position for the mouse ray @p ray.
    /// @param ray The mouse ray now.
    /// @return The position, or `std::nullopt` if the constraint is too nearly
    ///         edge-on or parallel to @p ray to follow it.
    [[nodiscard]] std::optional<glm::dvec3> Follow(const Ray& ray) const;

    /// The constraint the drag was last anchored with.
    /// @return The constraint.
    [[nodiscard]] DragConstraint Constraint() const { return m_constraint; }

    /// Whether the last Anchor fell back from the constraint asked for to the
    /// view plane, because that constraint was too nearly edge-on.
    /// @return True after a fallback.
    [[nodiscard]] bool FellBack() const { return m_fellBack; }

private:
    // Where ray meets the constraint through m_anchor.
    [[nodiscard]] std::optional<glm::dvec3> Hit(const Ray& ray) const;

    DragConstraint m_constraint = DragConstraint::ViewPlane;
    bool m_fellBack = false;
    glm::dvec3 m_anchor{0.0};
    // The plane normal, or for DragConstraint::Vertical the line direction.
    glm::dvec3 m_axis{0.0, 0.0, 1.0};
    // The point minus the hit at the anchor.
    glm::dvec3 m_offset{0.0};
};

} // namespace Graphics
