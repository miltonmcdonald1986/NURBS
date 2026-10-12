/// @file
/// RationalBezierCurve, a rational Bezier curve (Eq. 4.1).

#pragma once

#include <NURBS/bezier.hpp>
#include <NURBS/point.hpp>

#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <iterator>
#include <ranges>
#include <span>
#include <vector>

namespace NURBS
{

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
    requires PointLike<Point, Scalar>
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
