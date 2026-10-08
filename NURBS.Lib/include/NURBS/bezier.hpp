#pragma once

#include <NURBS/bernstein.hpp>
#include <NURBS/horner1.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <ranges>
#include <vector>

namespace NURBS
{

// Algorithm A1.4 (The NURBS Book): compute the point C(u) = sum B_{k,n}(u) * P[k],
// k = 0..n, on a Bezier curve, using AllBernstein to compute the basis functions.
// The degree n is P.size() - 1. Precondition: P is non-empty.
template <std::floating_point Scalar,
          std::ranges::random_access_range R,
          typename Point = std::ranges::range_value_t<R>>
    requires std::ranges::sized_range<R> && CurvePoint<Point, Scalar>
[[nodiscard]] constexpr Point PointOnBezierCurve(const R& P, Scalar u)
{
    assert(!std::ranges::empty(P));

    const auto first = std::ranges::begin(P);
    const auto n = std::ranges::size(P) - 1;

    const std::vector<Scalar> B = AllBernstein(n, u);
    Point C = B[0] * first[0];
    for (std::size_t k = 1; k <= n; ++k)
        C = C + B[k] * first[k];
    return C;
}

} // namespace NURBS
