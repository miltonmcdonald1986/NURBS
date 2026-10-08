#pragma once

#include <cassert>
#include <concepts>
#include <ranges>

namespace NURBS
{

// A point type usable as a power basis coefficient: closed under addition
// and under multiplication by a scalar of type S.
template <typename P, typename S>
concept CurvePoint = std::copyable<P> && requires(const P p, const P q, const S s) {
    { p + q } -> std::convertible_to<P>;
    { s * p } -> std::convertible_to<P>;
};

// Algorithm A1.1 (The NURBS Book): compute the point C(u0) = sum a[i] * u0^i,
// i = 0..n, on a power basis curve using Horner's method. The degree n is
// a.size() - 1. Precondition: a is non-empty.
template <std::floating_point Scalar,
          std::ranges::random_access_range R,
          typename Point = std::ranges::range_value_t<R>>
    requires std::ranges::sized_range<R> && CurvePoint<Point, Scalar>
[[nodiscard]] constexpr Point horner1(const R& a, Scalar u0)
{
    assert(!std::ranges::empty(a));

    const auto first = std::ranges::begin(a);
    const auto n = std::ranges::size(a) - 1;

    Point C = first[n];
    for (auto i = n; i-- > 0;)
        C = u0 * C + first[i];
    return C;
}

} // namespace NURBS
