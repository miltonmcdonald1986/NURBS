/// @file
/// PointDrag, which drags one point in 3D under a constraint.

#pragma once

#include "Picking.hpp"

#include <glm/vec3.hpp>

#include <cstdint>
#include <optional>

namespace Graphics
{

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
