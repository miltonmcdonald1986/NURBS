#pragma once

#include <glm/common.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>

namespace NURBS::Viewer
{

// Evaluates eval(u) at count evenly spaced parameters from u0 to u1 inclusive.
// The points have eval's result type, e.g. glm::dvec2 or glm::dvec3.
// Precondition: count >= 2.
template <typename Eval, typename Point = std::remove_cvref_t<std::invoke_result_t<Eval&, double>>>
    requires std::invocable<Eval&, double>
[[nodiscard]] std::vector<Point> SampleCurve(Eval&& eval, double u0, double u1, std::size_t count)
{
    assert(count >= 2);

    std::vector<Point> points;
    points.reserve(count);
    const double last = static_cast<double>(count - 1);
    for (std::size_t i = 0; i < count; ++i)
        points.push_back(eval(u0 + (u1 - u0) * (static_cast<double>(i) / last)));
    return points;
}

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

} // namespace NURBS::Viewer
