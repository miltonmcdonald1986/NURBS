#pragma once

#include <glm/common.hpp>
#include <glm/vec2.hpp>

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
// Precondition: count >= 2.
template <typename Eval>
    requires std::invocable<Eval&, double> && std::convertible_to<std::invoke_result_t<Eval&, double>, glm::dvec2>
[[nodiscard]] std::vector<glm::dvec2> SampleCurve(Eval&& eval, double u0, double u1, std::size_t count)
{
    assert(count >= 2);

    std::vector<glm::dvec2> points;
    points.reserve(count);
    const double last = static_cast<double>(count - 1);
    for (std::size_t i = 0; i < count; ++i)
        points.push_back(eval(u0 + (u1 - u0) * (static_cast<double>(i) / last)));
    return points;
}

struct Bounds2D
{
    glm::dvec2 min;
    glm::dvec2 max;

    void Extend(glm::dvec2 p)
    {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }

    void Extend(std::span<const glm::dvec2> points)
    {
        for (const glm::dvec2& p : points)
            Extend(p);
    }
};

// The axis-aligned bounding box of points, or nullopt if points is empty.
[[nodiscard]] inline std::optional<Bounds2D> BoundsOf(std::span<const glm::dvec2> points)
{
    if (points.empty())
        return std::nullopt;

    Bounds2D bounds{points.front(), points.front()};
    bounds.Extend(points);
    return bounds;
}

} // namespace NURBS::Viewer
