/// @file
/// Bezier curve and surface evaluation (A1.4, A1.5, A1.7).

#pragma once

#include <NURBS/bernstein.hpp>
#include <NURBS/point.hpp>

#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <iterator>
#include <ranges>
#include <vector>

namespace NURBS
{

/// Computes a point on a Bezier curve from its basis functions
/// (Algorithm A1.4, *The NURBS Book*).
///
/// Evaluates \f$C(u) = \sum_{k=0}^{n} B_{k,n}(u) P_k\f$, using AllBernstein()
/// to compute the basis functions \f$B_{k,n}\f$.
/// @tparam Scalar The floating-point parameter type.
/// @tparam R A random-access, sized range of control points.
/// @tparam Point The control point type, deduced from @p R.
/// @param P The control points \f$P_0, \ldots, P_n\f$. The degree is
///          \f$n = \f$ `P.size() - 1`.
/// @param u The parameter value to evaluate at, normally in \f$[0, 1]\f$.
/// @return The point \f$C(u)\f$.
/// @pre @p P is non-empty.
template <std::floating_point Scalar, std::ranges::random_access_range R,
          typename Point = std::ranges::range_value_t<R>>
    requires std::ranges::sized_range<R> && PointLike<Point, Scalar>
[[nodiscard]] constexpr Point PointOnBezierCurve(const R& P, Scalar u)
{
    assert(!std::ranges::empty(P));

    const auto first = std::ranges::begin(P);
    const auto n = std::ranges::size(P) - 1;

    using Diff = std::ranges::range_difference_t<R>;

    const std::vector<Scalar> B = AllBernstein(n, u);
    Point C = B[0] * first[0];
    for (std::size_t k = 1; k <= n; ++k)
        C = C + B[k] * first[static_cast<Diff>(k)];
    return C;
}

/// Computes a point on a Bezier curve with de Casteljau's algorithm
/// (Algorithm A1.5, *The NURBS Book*).
///
/// Starting from \f$Q_i = P_i\f$, repeatedly applies the linear interpolation
/// \f$Q_i = (1-u) Q_i + u Q_{i+1}\f$ until one point remains. Every step is a
/// convex combination when \f$u \in [0, 1]\f$, which makes this more
/// numerically stable than PointOnBezierCurve(), at a cost of
/// \f$O(n^2)\f$ operations.
/// @tparam Scalar The floating-point parameter type.
/// @tparam R A random-access, sized range of control points.
/// @tparam Point The control point type, deduced from @p R.
/// @param P The control points \f$P_0, \ldots, P_n\f$. The degree is
///          \f$n = \f$ `P.size() - 1`.
/// @param u The parameter value to evaluate at, normally in \f$[0, 1]\f$.
/// @return The point \f$C(u)\f$.
/// @pre @p P is non-empty.
template <std::floating_point Scalar, std::ranges::random_access_range R,
          typename Point = std::ranges::range_value_t<R>>
    requires std::ranges::sized_range<R> && PointLike<Point, Scalar>
[[nodiscard]] constexpr Point deCasteljau1(const R& P, Scalar u)
{
    assert(!std::ranges::empty(P));

    const auto n = std::ranges::size(P) - 1;

    // Not the iterator-pair constructor: views such as the columns built by
    // deCasteljau2() need not be C++17 iterators (libstdc++'s iota_view uses
    // an integer-class difference type), which that constructor requires.
    std::vector<Point> Q;
    Q.reserve(n + 1);
    std::ranges::copy(P, std::back_inserter(Q));
    const Scalar u1 = Scalar{1} - u;
    for (std::size_t k = 1; k <= n; ++k)
    {
        for (std::size_t i = 0; i <= n - k; ++i)
        {
            Q[i] = u1 * Q[i] + u * Q[i + 1];
        }
    }
    return Q[0];
}

/// Computes a point on a Bezier surface with de Casteljau's algorithm
/// (Algorithm A1.7, *The NURBS Book*).
///
/// Evaluates
/// \f$S(u, v) = \sum_{i=0}^{n} \sum_{j=0}^{m} B_{i,n}(u) B_{j,m}(v) P_{i,j}\f$.
/// Each row \f$i\f$ is a Bezier curve in \f$v\f$; evaluating it at \f$v\f$
/// with deCasteljau1() gives \f$Q_i\f$, the control points of the
/// isoparametric curve \f$S(u, v) = \sum_{i=0}^{n} B_{i,n}(u) Q_i\f$, which is
/// then evaluated at \f$u\f$. Equally, each column can be evaluated at \f$u\f$
/// first, then the resulting curve at \f$v\f$. The first order costs
/// \f$O(n m^2 + n^2)\f$ operations and the second \f$O(m n^2 + m^2)\f$, so,
/// as in the book, the columns go first when \f$n \le m\f$.
/// @tparam Scalar The floating-point parameter type.
/// @tparam R A random-access, sized range of rows, each itself a
///           random-access, sized range of control points.
/// @tparam Row The row type, deduced from @p R.
/// @tparam Point The control point type, deduced from @p Row.
/// @param P The control points, indexed `P[i][j]`. The degree in \f$u\f$ is
///          \f$n = \f$ `P.size() - 1` and the degree in \f$v\f$ is
///          \f$m = \f$ `P[0].size() - 1`.
/// @param u The \f$u\f$ parameter value to evaluate at, normally in
///          \f$[0, 1]\f$.
/// @param v The \f$v\f$ parameter value to evaluate at, normally in
///          \f$[0, 1]\f$.
/// @return The point \f$S(u, v)\f$.
/// @pre @p P is non-empty, its rows are non-empty, and all rows have the
///      same size.
template <std::floating_point Scalar, std::ranges::random_access_range R, typename Row = std::ranges::range_value_t<R>,
          typename Point = std::ranges::range_value_t<Row>>
    requires std::ranges::sized_range<R> && std::ranges::random_access_range<Row> && std::ranges::sized_range<Row> &&
             PointLike<Point, Scalar>
[[nodiscard]] constexpr Point deCasteljau2(const R& P, Scalar u, Scalar v)
{
    assert(!std::ranges::empty(P));
    assert(!std::ranges::empty(*std::ranges::begin(P)));
    assert(std::ranges::all_of(
        P, [&P](const auto& row) { return std::ranges::size(row) == std::ranges::size(*std::ranges::begin(P)); }));

    const auto first = std::ranges::begin(P);
    const auto n = std::ranges::size(P) - 1;
    const auto m = std::ranges::size(*first) - 1;

    std::vector<Point> Q;
    if (n <= m)
    {
        // Q[j] = C_j(u), where C_j is the curve through column j.
        Q.reserve(m + 1);
        for (std::size_t j = 0; j <= m; ++j)
        {
            using Diff = std::ranges::range_difference_t<Row>;
            const auto column = std::views::transform(
                P, [j](const auto& row) -> Point { return std::ranges::begin(row)[static_cast<Diff>(j)]; });
            Q.push_back(deCasteljau1(column, u));
        }
        return deCasteljau1(Q, v);
    }

    // Q[i] = C_i(v), where C_i is the curve through row i.
    Q.reserve(n + 1);
    for (const auto& row : P)
        Q.push_back(deCasteljau1(row, v));
    return deCasteljau1(Q, u);
}

} // namespace NURBS
