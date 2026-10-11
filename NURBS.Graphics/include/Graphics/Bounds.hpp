#pragma once

#include <glm/common.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <optional>
#include <span>

namespace Graphics
{

// An axis-aligned box, for Vec = glm::dvec2 or glm::dvec3.
template <typename Vec>
struct Bounds
{
    Vec min;
    Vec max;

    void Extend(Vec p)
    {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }

    void Extend(std::span<const Vec> points)
    {
        for (const Vec& p : points)
            Extend(p);
    }
};

using Bounds2D = Bounds<glm::dvec2>;
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

// The axis-aligned bounding box of points, or nullopt if points is empty.
// (Overloads rather than a template, so a std::vector converts to the span.)
[[nodiscard]] inline std::optional<Bounds2D> BoundsOf(std::span<const glm::dvec2> points)
{
    return Detail::BoundsOf(points);
}

[[nodiscard]] inline std::optional<Bounds3D> BoundsOf(std::span<const glm::dvec3> points)
{
    return Detail::BoundsOf(points);
}

} // namespace Graphics
