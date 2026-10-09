#pragma once

#include "Viewport2D.hpp"

#include <glm/vec2.hpp>

#include <cstddef>
#include <span>
#include <vector>

namespace NURBS::Viewer
{

// Shared by the scenes whose curves are defined by a control polygon
// P[0], P[1], ..., P[n] (Bezier and rational Bezier).

// The control points for a degree slider set to count - 1: the first count
// of points, with points appended by NextControlPoint while there are fewer.
// Precondition: points is non-empty and count >= 1.
[[nodiscard]] std::vector<glm::dvec2> ResizeControlPolygon(std::span<const glm::dvec2> points, std::size_t count);

// The lines P[0], P[1], ..., P[n].
void DrawControlPolygon(const Viewport2D& view, std::span<const glm::dvec2> points);

// The control points labeled P0..Pn, with the one at index dragged (-1 for
// none) highlighted. When weights is non-empty, each label also shows its
// weight, e.g. "P1 w=0.71". Precondition: weights is empty or as long as points.
void DrawControlPoints(const Viewport2D& view, std::span<const glm::dvec2> points, int dragged,
                       std::span<const double> weights = {});

} // namespace NURBS::Viewer
