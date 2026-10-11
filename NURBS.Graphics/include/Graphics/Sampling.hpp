/// @file
/// Evenly spaced parameters and curve sampling.

#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <type_traits>
#include <vector>

namespace Graphics
{

/// The @p i-th of @p count evenly spaced parameters from @p min to @p max inclusive.
/// @param min The first parameter.
/// @param max The last parameter.
/// @param i The index of the parameter, in [0, @p count).
/// @param count The number of parameters.
/// @return The parameter at index @p i.
/// @pre @p count >= 2.
[[nodiscard]] inline double ParameterAt(double min, double max, std::size_t i, std::size_t count)
{
    assert(count >= 2);
    return min + (max - min) * (static_cast<double>(i) / static_cast<double>(count - 1));
}

/// Evaluates @p eval at @p count evenly spaced parameters from @p u0 to @p u1 inclusive.
/// @tparam Eval A callable taking a `double` parameter.
/// @tparam Point The point type, deduced from @p Eval's result, e.g. `glm::dvec2` or `glm::dvec3`.
/// @param eval The curve, as a function of its parameter.
/// @param u0 The first parameter.
/// @param u1 The last parameter.
/// @param count The number of points.
/// @return The points \f$eval(u)\f$ in order of increasing index.
/// @pre @p count >= 2.
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
