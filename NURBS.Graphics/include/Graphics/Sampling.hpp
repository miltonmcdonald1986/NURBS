#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <type_traits>
#include <vector>

namespace Graphics
{

// The i-th of count evenly spaced parameters from min to max inclusive.
// Precondition: count >= 2.
[[nodiscard]] inline double ParameterAt(double min, double max, std::size_t i, std::size_t count)
{
    assert(count >= 2);
    return min + (max - min) * (static_cast<double>(i) / static_cast<double>(count - 1));
}

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
    for (std::size_t i = 0; i < count; ++i)
        points.push_back(eval(ParameterAt(u0, u1, i, count)));
    return points;
}

} // namespace Graphics
