/// @file
/// B-spline basis functions (A2.1).

#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <ranges>

namespace NURBS
{

/// Determines the knot span index (Algorithm A2.1, *The NURBS Book*).
///
/// Finds the index \f$i\f$ with \f$u_i \le u < u_{i+1}\f$ by binary search over
/// the knot vector \f$U = \{u_0, \ldots, u_m\}\f$. The number of basis
/// functions follows from \f$m = n + p + 1\f$, so the book's parameter \f$n\f$
/// is computed from the size of @p U rather than passed in.
///
/// Only the spans \f$p, \ldots, n\f$ make up the parameter domain
/// \f$[u_p, u_{n+1}]\f$, for clamped and unclamped knot vectors alike. At its
/// end, \f$u = u_{n+1}\f$, the span is taken to be \f$n\f$, so the result is
/// always in \f$[p, n]\f$.
/// @tparam Scalar The floating-point type of @p u and the knots.
/// @tparam R A random-access, sized range of knots.
/// @param p The degree.
/// @param u The parameter value to find the span of.
/// @param U The knot vector \f$u_0, \ldots, u_m\f$, non-decreasing.
/// @return The knot span index \f$i\f$.
/// @pre `U.size() >= 2 * p + 2` (at least \f$p + 1\f$ basis functions) and
///      \f$u_p \le u \le u_{n+1}\f$, where \f$n = \f$ `U.size() - p - 2`.
template <std::floating_point Scalar, std::ranges::random_access_range R>
    requires std::ranges::sized_range<R> && std::same_as<std::ranges::range_value_t<R>, Scalar>
[[nodiscard]] constexpr std::size_t FindSpan(std::size_t p, Scalar u, const R& U)
{
    assert(std::ranges::size(U) >= 2 * p + 2);

    const auto n = static_cast<std::size_t>(std::ranges::size(U)) - p - 2;
    const auto first = std::ranges::begin(U);
    using Diff = std::ranges::range_difference_t<R>;
    const auto knot = [first](std::size_t k) { return first[static_cast<Diff>(k)]; };

    assert(knot(p) <= u && u <= knot(n + 1));

    if (u == knot(n + 1)) // Special case
        return n;

    std::size_t low = p;
    std::size_t high = n + 1;
    std::size_t mid = (low + high) / 2;
    while (u < knot(mid) || u >= knot(mid + 1))
    {
        if (u < knot(mid))
            high = mid;
        else
            low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

} // namespace NURBS
