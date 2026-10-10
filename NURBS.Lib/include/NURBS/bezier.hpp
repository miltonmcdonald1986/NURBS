/// @file
/// Bezier curve evaluation (A1.4, A1.5) and the BezierCurve and
/// RationalBezierCurve classes.

#pragma once

#include <NURBS/bernstein.hpp>
#include <NURBS/point.hpp>

#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <ranges>
#include <span>
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
template <std::floating_point Scalar,
          std::ranges::random_access_range R,
          typename Point = std::ranges::range_value_t<R>>
    requires std::ranges::sized_range<R> && CurvePoint<Point, Scalar>
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
template <std::floating_point Scalar,
          std::ranges::random_access_range R,
          typename Point = std::ranges::range_value_t<R>>
    requires std::ranges::sized_range<R> && CurvePoint<Point, Scalar>
[[nodiscard]] constexpr Point deCasteljau1(const R& P, Scalar u)
{
    assert(!std::ranges::empty(P));

    const auto n = std::ranges::size(P) - 1;

    std::vector<Point> Q(std::ranges::begin(P), std::ranges::end(P));
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

/// A Bezier curve \f$C(u) = \sum_{k=0}^{n} B_{k,n}(u) P_k\f$ of degree
/// \f$n\f$, defined by \f$n + 1\f$ control points.
///
/// The control points are stored by value. Evaluate() uses de Casteljau's
/// algorithm (deCasteljau1()).
/// @tparam Point The control point type.
/// @tparam Scalar The floating-point parameter type, by default
///         ScalarOf<Point>.
template <typename Point, std::floating_point Scalar = ScalarOf<Point>>
    requires CurvePoint<Point, Scalar>
class BezierCurve
{
public:
    /// Constructs a curve from a range of control points.
    /// @tparam R An input range whose elements convert to @p Point.
    /// @param controlPoints The control points \f$P_0, \ldots, P_n\f$.
    /// @pre @p controlPoints is non-empty.
    template <std::ranges::input_range R>
        requires std::convertible_to<std::ranges::range_reference_t<R>, Point>
    explicit constexpr BezierCurve(const R& controlPoints)
        : m_controlPoints(std::ranges::begin(controlPoints), std::ranges::end(controlPoints))
    {
        assert(!m_controlPoints.empty());
    }

    /// Constructs a curve from a list of control points.
    /// @param controlPoints The control points \f$P_0, \ldots, P_n\f$.
    /// @pre @p controlPoints is non-empty.
    constexpr BezierCurve(std::initializer_list<Point> controlPoints)
        : m_controlPoints(controlPoints)
    {
        assert(!m_controlPoints.empty());
    }

    /// Computes a point on the curve with de Casteljau's algorithm.
    /// @param u The parameter value to evaluate at, normally in \f$[0, 1]\f$.
    /// @return The point \f$C(u)\f$.
    [[nodiscard]] constexpr Point Evaluate(Scalar u) const { return deCasteljau1(m_controlPoints, u); }

    /// Returns the degree of the curve.
    /// @return The degree \f$n\f$, one less than the number of control points.
    [[nodiscard]] constexpr std::size_t Degree() const { return m_controlPoints.size() - 1; }

    /// Returns the control points.
    /// @return A read-only view of \f$P_0, \ldots, P_n\f$.
    [[nodiscard]] constexpr std::span<const Point> ControlPoints() const { return m_controlPoints; }

    /// Returns the control points for editing in place.
    ///
    /// The degree is fixed, so changing it means constructing a new curve.
    /// @return A mutable view of \f$P_0, \ldots, P_n\f$.
    [[nodiscard]] constexpr std::span<Point> ControlPoints() { return m_controlPoints; }

private:
    std::vector<Point> m_controlPoints;
};

/// Deduces the point type of a BezierCurve from a range of control points.
/// @tparam R An input range of control points.
template <std::ranges::input_range R>
BezierCurve(const R&) -> BezierCurve<std::ranges::range_value_t<R>>;

/// A rational Bezier curve
/// \f$C(u) = \frac{\sum_{k=0}^{n} B_{k,n}(u) w_k P_k}{\sum_{k=0}^{n} B_{k,n}(u) w_k}\f$
/// of degree \f$n\f$ (Eq. 4.1, *The NURBS Book*).
///
/// It is a polynomial Bezier curve in homogeneous space,
/// \f$P^w_k = (w_k P_k, w_k)\f$, projected back by dividing by the last
/// coordinate. Since de Casteljau works on each coordinate independently,
/// Evaluate() runs deCasteljau1() separately on the weighted points
/// \f$w_k P_k\f$ and on the weights, so no homogeneous point type is needed.
/// @tparam Point The control point type.
/// @tparam Scalar The floating-point type of the parameter and the weights,
///         by default ScalarOf<Point>.
/// @invariant There is one weight per control point, and every weight is
///            positive.
template <typename Point, std::floating_point Scalar = ScalarOf<Point>>
    requires CurvePoint<Point, Scalar>
class RationalBezierCurve
{
public:
    /// Constructs a curve with all weights 1, which is the same curve as a
    /// BezierCurve on the same control points.
    /// @tparam R An input range whose elements convert to @p Point.
    /// @param controlPoints The control points \f$P_0, \ldots, P_n\f$.
    /// @pre @p controlPoints is non-empty.
    template <std::ranges::input_range R>
        requires std::convertible_to<std::ranges::range_reference_t<R>, Point>
    explicit constexpr RationalBezierCurve(const R& controlPoints)
        : m_controlPoints(std::ranges::begin(controlPoints), std::ranges::end(controlPoints)),
          m_weights(m_controlPoints.size(), Scalar{1})
    {
        assert(!m_controlPoints.empty());
    }

    /// Constructs a curve from control points and their weights.
    /// @tparam R An input range whose elements convert to @p Point.
    /// @tparam W An input range whose elements convert to @p Scalar.
    /// @param controlPoints The control points \f$P_0, \ldots, P_n\f$.
    /// @param weights The weights \f$w_0, \ldots, w_n\f$.
    /// @pre @p controlPoints is non-empty, @p weights has the same size, and
    ///      every weight is positive.
    template <std::ranges::input_range R, std::ranges::input_range W>
        requires std::convertible_to<std::ranges::range_reference_t<R>, Point> &&
                 std::convertible_to<std::ranges::range_reference_t<W>, Scalar>
    constexpr RationalBezierCurve(const R& controlPoints, const W& weights)
        : m_controlPoints(std::ranges::begin(controlPoints), std::ranges::end(controlPoints)),
          m_weights(std::ranges::begin(weights), std::ranges::end(weights))
    {
        assert(!m_controlPoints.empty());
        assert(m_weights.size() == m_controlPoints.size());
        assert(std::ranges::all_of(m_weights, [](Scalar w) { return w > Scalar{0}; }));
    }

    /// Computes a point on the curve with de Casteljau's algorithm in
    /// homogeneous space.
    /// @param u The parameter value to evaluate at, normally in \f$[0, 1]\f$.
    /// @return The point \f$C(u)\f$.
    [[nodiscard]] constexpr Point Evaluate(Scalar u) const
    {
        // The first coordinates of the homogeneous points Pw[k]: w[k] * P[k].
        std::vector<Point> weightedPoints;
        weightedPoints.reserve(m_controlPoints.size());
        std::ranges::transform(m_weights, m_controlPoints, std::back_inserter(weightedPoints),
                               [](Scalar w, const Point& p) -> Point { return w * p; });

        // Evaluate both parts of Cw(u), then project back by dividing by its weight.
        const Point numerator = deCasteljau1(weightedPoints, u);
        const Scalar denominator = deCasteljau1(m_weights, u);
        return (Scalar{1} / denominator) * numerator;
    }

    /// Returns the degree of the curve.
    /// @return The degree \f$n\f$, one less than the number of control points.
    [[nodiscard]] constexpr std::size_t Degree() const { return m_controlPoints.size() - 1; }

    /// Returns the control points.
    /// @return A read-only view of \f$P_0, \ldots, P_n\f$.
    [[nodiscard]] constexpr std::span<const Point> ControlPoints() const { return m_controlPoints; }

    /// Returns the control points for editing in place.
    ///
    /// The degree is fixed, so changing it means constructing a new curve.
    /// @return A mutable view of \f$P_0, \ldots, P_n\f$.
    [[nodiscard]] constexpr std::span<Point> ControlPoints() { return m_controlPoints; }

    /// Returns the weights.
    /// @return A read-only view of \f$w_0, \ldots, w_n\f$.
    [[nodiscard]] constexpr std::span<const Scalar> Weights() const { return m_weights; }

    /// Returns the weights for editing in place.
    ///
    /// Edited weights must stay positive.
    /// @return A mutable view of \f$w_0, \ldots, w_n\f$.
    [[nodiscard]] constexpr std::span<Scalar> Weights() { return m_weights; }

private:
    std::vector<Point> m_controlPoints;
    std::vector<Scalar> m_weights;
};

/// Deduces the point type of a RationalBezierCurve from a range of control
/// points.
/// @tparam R An input range of control points.
template <std::ranges::input_range R>
RationalBezierCurve(const R&) -> RationalBezierCurve<std::ranges::range_value_t<R>>;

/// Deduces the point type of a RationalBezierCurve from a range of control
/// points and a range of weights.
/// @tparam R An input range of control points.
/// @tparam W An input range of weights.
template <std::ranges::input_range R, std::ranges::input_range W>
RationalBezierCurve(const R&, const W&) -> RationalBezierCurve<std::ranges::range_value_t<R>>;

} // namespace NURBS
