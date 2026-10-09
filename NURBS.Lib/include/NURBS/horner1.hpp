/// @file
/// Power basis curve evaluation (A1.1) and the CurvePoint concept.

#pragma once

#include <cassert>
#include <concepts>
#include <ranges>

namespace NURBS
{

/// A point type usable as a curve coefficient or control point: copyable,
/// closed under addition, and closed under multiplication by a scalar of
/// type @p S.
/// @tparam P The point type, e.g. `double` or `glm::dvec3`.
/// @tparam S The scalar type the point is multiplied by.
template <typename P, typename S>
concept CurvePoint = std::copyable<P> && requires(const P p, const P q, const S s) {
    { p + q } -> std::convertible_to<P>;
    { s * p } -> std::convertible_to<P>;
};

/// Computes a point on a power basis curve with Horner's method
/// (Algorithm A1.1, *The NURBS Book*).
///
/// Evaluates \f$C(u_0) = \sum_{i=0}^{n} a_i u_0^i\f$ as
/// \f$(\cdots(a_n u_0 + a_{n-1}) u_0 + \cdots) u_0 + a_0\f$.
/// @tparam Scalar The floating-point parameter type.
/// @tparam R A random-access, sized range of coefficients.
/// @tparam Point The coefficient type, deduced from @p R.
/// @param a The coefficients \f$a_0, \ldots, a_n\f$. The degree is
///          \f$n = \f$ `a.size() - 1`.
/// @param u0 The parameter value to evaluate at.
/// @return The point \f$C(u_0)\f$.
/// @pre @p a is non-empty.
template <std::floating_point Scalar,
          std::ranges::random_access_range R,
          typename Point = std::ranges::range_value_t<R>>
    requires std::ranges::sized_range<R> && CurvePoint<Point, Scalar>
[[nodiscard]] constexpr Point horner1(const R& a, Scalar u0)
{
    assert(!std::ranges::empty(a));

    const auto first = std::ranges::begin(a);
    // Signed, to index the iterator without a sign conversion.
    const auto n = std::ranges::distance(a) - 1;

    Point C = first[n];
    for (auto i = n; i-- > 0;)
        C = u0 * C + first[i];
    return C;
}

} // namespace NURBS
