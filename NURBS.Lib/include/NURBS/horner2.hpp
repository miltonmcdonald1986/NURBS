/// @file
/// Power basis surface evaluation (A1.6).

#pragma once

#include <NURBS/horner1.hpp>

#include <algorithm>
#include <cassert>
#include <concepts>
#include <ranges>

namespace NURBS
{

/// Computes a point on a power basis surface with Horner's method
/// (Algorithm A1.6, *The NURBS Book*).
///
/// Evaluates \f$S(u_0, v_0) = \sum_{i=0}^{n} \sum_{j=0}^{m} a_{i,j} u_0^i v_0^j\f$.
/// Each row \f$i\f$ is a power basis curve in \f$v\f$; evaluating it at
/// \f$v_0\f$ with horner1() gives \f$b_i\f$, the coefficients of the
/// isoparametric curve \f$S(u, v_0) = \sum_{i=0}^{n} b_i u^i\f$, which is then
/// evaluated at \f$u_0\f$. The book stores the \f$b_i\f$ in an array; here
/// each one is folded into the outer Horner chain as soon as it is computed,
/// so no storage is needed.
/// @tparam Scalar The floating-point parameter type.
/// @tparam R A random-access, sized range of rows, each itself a
///           random-access, sized range of coefficients.
/// @tparam Row The row type, deduced from @p R.
/// @tparam Point The coefficient type, deduced from @p Row.
/// @param a The coefficients, indexed `a[i][j]` for \f$u^i v^j\f$. The
///          degree in \f$u\f$ is \f$n = \f$ `a.size() - 1` and the degree in
///          \f$v\f$ is \f$m = \f$ `a[0].size() - 1`.
/// @param u0 The \f$u\f$ parameter value to evaluate at.
/// @param v0 The \f$v\f$ parameter value to evaluate at.
/// @return The point \f$S(u_0, v_0)\f$.
/// @pre @p a is non-empty, its rows are non-empty, and all rows have the
///      same size.
template <std::floating_point Scalar,
          std::ranges::random_access_range R,
          typename Row = std::ranges::range_value_t<R>,
          typename Point = std::ranges::range_value_t<Row>>
    requires std::ranges::sized_range<R> && std::ranges::random_access_range<Row> &&
             std::ranges::sized_range<Row> && CurvePoint<Point, Scalar>
[[nodiscard]] constexpr Point horner2(const R& a, Scalar u0, Scalar v0)
{
    assert(!std::ranges::empty(a));
    assert(std::ranges::all_of(a, [&a](const auto& row) {
        return std::ranges::size(row) == std::ranges::size(*std::ranges::begin(a));
    }));

    const auto first = std::ranges::begin(a);
    // Signed, to index the iterator without a sign conversion.
    const auto n = std::ranges::distance(a) - 1;

    Point S = horner1(first[n], v0);
    for (auto i = n; i-- > 0;)
        S = u0 * S + horner1(first[i], v0);
    return S;
}

} // namespace NURBS
