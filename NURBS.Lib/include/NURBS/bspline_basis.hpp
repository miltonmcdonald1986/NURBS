/// @file
/// B-spline basis functions (A2.1, A2.2).

#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <ranges>
#include <vector>

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

/// Computes the nonvanishing basis functions (Algorithm A2.2, *The NURBS Book*).
///
/// On the knot span \f$[u_i, u_{i+1})\f$ at most \f$p + 1\f$ of the degree
/// \f$p\f$ basis functions are nonzero, namely
/// \f$N_{i-p,p}(u), \ldots, N_{i,p}(u)\f$. They are computed together from
/// the Cox-de Boor recurrence, sharing intermediate terms and avoiding any
/// division by zero.
/// @tparam Scalar The floating-point type of @p u, the knots and the results.
/// @tparam R A random-access, sized range of knots.
/// @param i The knot span index, as returned by FindSpan().
/// @param u The parameter value to evaluate at.
/// @param p The degree.
/// @param U The knot vector \f$u_0, \ldots, u_m\f$, non-decreasing.
/// @return A vector `N` of size \f$p + 1\f$ with `N[j]` \f$= N_{i-p+j,p}(u)\f$.
/// @pre `U.size() >= 2 * p + 2`, \f$p \le i \le n\f$ and
///      \f$u_i \le u \le u_{i+1}\f$, where \f$n = \f$ `U.size() - p - 2`.
template <std::floating_point Scalar, std::ranges::random_access_range R>
    requires std::ranges::sized_range<R> && std::same_as<std::ranges::range_value_t<R>, Scalar>
[[nodiscard]] constexpr std::vector<Scalar> BasisFuns(std::size_t i, Scalar u, std::size_t p, const R& U)
{
    assert(std::ranges::size(U) >= 2 * p + 2);
    assert(p <= i && i <= static_cast<std::size_t>(std::ranges::size(U)) - p - 2);

    const auto first = std::ranges::begin(U);
    using Diff = std::ranges::range_difference_t<R>;
    const auto knot = [first](std::size_t k) { return first[static_cast<Diff>(k)]; };

    assert(knot(i) <= u && u <= knot(i + 1));

    std::vector<Scalar> N(p + 1, Scalar{0});
    std::vector<Scalar> left(p + 1, Scalar{0});
    std::vector<Scalar> right(p + 1, Scalar{0});
    N[0] = Scalar{1};
    for (std::size_t j = 1; j <= p; ++j)
    {
        left[j] = u - knot(i + 1 - j);
        right[j] = knot(i + j) - u;
        Scalar saved{0};
        for (std::size_t r = 0; r < j; ++r)
        {
            const Scalar temp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
    return N;
}

} // namespace NURBS
