/// @file
/// Axis-aligned bounding boxes.

#pragma once

#include <glm/common.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <optional>
#include <span>

namespace Graphics
{

/// An axis-aligned box.
/// @tparam Vec The point type, `glm::dvec2` or `glm::dvec3`.
template <typename Vec>
struct Bounds
{
    /// The minimum corner.
    Vec min;
    /// The maximum corner.
    Vec max;

    /// Grows the box to contain @p p.
    /// @param p The point to contain.
    void Extend(Vec p)
    {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }

    /// Grows the box to contain every point in @p points.
    /// @param points The points to contain.
    void Extend(std::span<const Vec> points)
    {
        for (const Vec& p : points)
            Extend(p);
    }
};

/// A 2D axis-aligned box.
using Bounds2D = Bounds<glm::dvec2>;
/// A 3D axis-aligned box.
using Bounds3D = Bounds<glm::dvec3>;

namespace Detail
{

template <typename Vec>
[[nodiscard]] std::optional<Bounds<Vec>> BoundsOf(std::span<const Vec> points)
{
    if (points.empty())
        return std::nullopt;

    Bounds<Vec> bounds{points.front(), points.front()};
    bounds.Extend(points);
    return bounds;
}

} // namespace Detail

/// The axis-aligned bounding box of 2D points.
/// (Overloads rather than a template, so a std::vector converts to the span.)
/// @param points The points.
/// @return The bounding box, or `std::nullopt` if @p points is empty.
[[nodiscard]] inline std::optional<Bounds2D> BoundsOf(std::span<const glm::dvec2> points)
{
    return Detail::BoundsOf(points);
}

/// The axis-aligned bounding box of 3D points.
/// @param points The points.
/// @return The bounding box, or `std::nullopt` if @p points is empty.
[[nodiscard]] inline std::optional<Bounds3D> BoundsOf(std::span<const glm::dvec3> points)
{
    return Detail::BoundsOf(points);
}

} // namespace Graphics
