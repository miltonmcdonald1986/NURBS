/// @file
/// B-spline curve evaluation (A3.1).

#pragma once

#include <NURBS/bspline_basis.hpp>
#include <NURBS/point.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <ranges>
#include <vector>

namespace NURBS
{

/// Computes a point on a B-spline curve (Algorithm A3.1, *The NURBS Book*).
///
/// Evaluates \f$C(u) = \sum_{i=0}^{n} N_{i,p}(u) P_i\f$. Only the \f$p + 1\f$
/// basis functions that are nonzero on the knot span of @p u contribute, so
/// the span is found with FindSpan(), those functions are computed with
/// BasisFuns(), and the sum runs over \f$P_{\text{span}-p}, \ldots,
/// P_{\text{span}}\f$ alone. The number of control points follows from
/// \f$m = n + p + 1\f$, so the book's parameter \f$n\f$ is computed from the
/// size of @p P rather than passed in.
///
/// At the end of the domain, \f$u = u_{n+1}\f$, the span is taken to be
/// \f$n\f$, as in FindSpan(), so a clamped curve ends at \f$P_n\f$.
/// @tparam Scalar The floating-point type of @p u and the knots.
/// @tparam RU A random-access, sized range of knots.
/// @tparam RP A random-access, sized range of control points.
/// @tparam Point The control point type, deduced from @p RP.
/// @param p The degree.
/// @param U The knot vector \f$u_0, \ldots, u_m\f$, non-decreasing.
/// @param P The control points \f$P_0, \ldots, P_n\f$.
/// @param u The parameter value to evaluate at.
/// @return The point \f$C(u)\f$.
/// @pre `P.size() >= p + 1`, `U.size() == P.size() + p + 1` and
///      \f$u_p \le u \le u_{n+1}\f$, where \f$n = \f$ `P.size() - 1`.
template <std::floating_point Scalar,
          std::ranges::random_access_range RU,
          std::ranges::random_access_range RP,
          typename Point = std::ranges::range_value_t<RP>>
    requires std::ranges::sized_range<RU> && std::same_as<std::ranges::range_value_t<RU>, Scalar> &&
             std::ranges::sized_range<RP> && PointLike<Point, Scalar>
[[nodiscard]] constexpr Point CurvePoint(std::size_t p, const RU& U, const RP& P, Scalar u)
{
    assert(std::ranges::size(P) >= p + 1);
    assert(std::ranges::size(U) == std::ranges::size(P) + p + 1);

    const auto first = std::ranges::begin(P);
    using Diff = std::ranges::range_difference_t<RP>;
    const auto point = [first](std::size_t k) -> Point { return first[static_cast<Diff>(k)]; };

    const std::size_t span = FindSpan(p, u, U);
    const std::vector<Scalar> N = BasisFuns(span, u, p, U);
    Point C = N[0] * point(span - p);
    for (std::size_t j = 1; j <= p; ++j)
        C = C + N[j] * point(span - p + j);
    return C;
}

} // namespace NURBS
