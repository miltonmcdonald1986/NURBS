#include "ControlPolygon.hpp"

#include "ImGuiHelpers.hpp"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <format>
#include <numbers>

namespace NURBS::Viewer
{

namespace
{

constexpr ImU32 kPolygonColor = IM_COL32(120, 120, 140, 200);
constexpr ImU32 kControlPointColor = IM_COL32(240, 180, 60, 255);
constexpr ImU32 kDraggedColor = IM_COL32(255, 255, 255, 255);

// How sharply each appended control point turns the control polygon, and how
// much shorter its new leg is than the previous one.
constexpr double kTurnRadians = std::numbers::pi / 4.0;
constexpr double kShrink = 0.75;
// The new leg when there is no previous one to follow.
constexpr glm::dvec2 kFirstLeg{0.5, 0.0};

// The control point appended when the degree slider raises the degree by one.
// Each new leg is the previous one turned toward the other points and shrunk,
// so repeated calls wind the polygon into an inward spiral: the curve keeps
// bending back, and since the leg lengths form a geometric series the points
// stay within 1 / (1 - kShrink) legs of where they started.
// Precondition: points is non-empty.
[[nodiscard]] glm::dvec2 NextControlPoint(std::span<const glm::dvec2> points)
{
    assert(!points.empty());

    const glm::dvec2 last = points.back();
    if (points.size() == 1)
        return last + kFirstLeg;
    const glm::dvec2 leg = last - points[points.size() - 2];
    if (glm::dot(leg, leg) == 0.0)
        return last + kFirstLeg;

    glm::dvec2 centroid{0.0, 0.0};
    for (const glm::dvec2& p : points)
        centroid += p;
    centroid /= static_cast<double>(points.size());

    // Turn counterclockwise if the centroid is left of the leg, else clockwise.
    const glm::dvec2 toCentroid = centroid - last;
    const double cross = leg.x * toCentroid.y - leg.y * toCentroid.x;
    const double angle = cross >= 0.0 ? kTurnRadians : -kTurnRadians;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const glm::dvec2 turned{c * leg.x - s * leg.y, s * leg.x + c * leg.y};
    return last + kShrink * turned;
}

// "P" followed by i and, if given, the weight, e.g. "P1 w=0.71".
[[nodiscard]] Label16 ControlPointLabel(std::size_t i, const double* weight)
{
    if (weight == nullptr)
        return IndexedLabel("P", i);

    Label16 label;
    std::format_to_n(label.text.data(), label.text.size() - 1, "P{} w={:.2f}", i, *weight);
    return label;
}

} // namespace

std::vector<glm::dvec2> ResizeControlPolygon(std::span<const glm::dvec2> points, std::size_t count)
{
    assert(!points.empty());
    assert(count >= 1);

    // TODO: When the library has Bezier degree elevation (The NURBS Book, Section 5.5),
    // raise the degree with it so the curve keeps its shape, and remove
    // NextControlPoint. Lowering the degree can keep truncating, since degree
    // reduction is only approximate.
    const std::span<const glm::dvec2> kept = points.first(std::min(count, points.size()));
    std::vector<glm::dvec2> resized(kept.begin(), kept.end());
    while (resized.size() < count)
    {
        const glm::dvec2 next = NextControlPoint(resized);
        resized.push_back(next);
    }
    return resized;
}

void DrawControlPolygon(const Viewport2D& view, std::span<const glm::dvec2> points)
{
    view.Polyline(points, kPolygonColor, 1.0f);
}

void DrawControlPoints(const Viewport2D& view, std::span<const glm::dvec2> points, int dragged,
                       std::span<const double> weights)
{
    assert(weights.empty() || weights.size() == points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const bool isDragged = static_cast<int>(i) == dragged;
        const double* weight = weights.empty() ? nullptr : &weights[i];
        view.LabeledPoint(points[i], ControlPointLabel(i, weight).c_str(), isDragged ? kDraggedColor : kControlPointColor,
                          isDragged ? 6.0f : 4.0f);
    }
}

} // namespace NURBS::Viewer
