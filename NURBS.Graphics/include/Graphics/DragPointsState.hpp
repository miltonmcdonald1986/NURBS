/// @file
/// DragPointsState, the state Viewport3D::DragPoints keeps between frames.

#pragma once

#include "PointDrag.hpp"

#include <glm/vec3.hpp>

namespace Graphics
{

/// The state of Viewport3D::DragPoints, kept by the caller between frames.
struct DragPointsState
{
    /// The index of the grabbed point, or -1 if none.
    int active = -1;
    /// The index of the point under the mouse while none is grabbed, or -1.
    /// Updated by every DragPoints call.
    int hovered = -1;
    /// Where the grabbed point was when it was grabbed.
    glm::dvec3 start{0.0};
    /// The constraint asked for by the modifier keys when last anchored.
    DragConstraint requested = DragConstraint::ViewPlane;
    /// Follows the mouse for the grabbed point.
    PointDrag drag;
    /// Escape put the grabbed point back; the left button is still held.
    bool cancelled = false;
};

} // namespace Graphics
