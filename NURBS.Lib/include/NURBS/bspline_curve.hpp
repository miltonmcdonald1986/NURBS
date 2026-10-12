/// @file
/// BSplineCurve, a nonrational B-spline curve.

#pragma once

#include <NURBS/bspline.hpp>
#include <NURBS/point.hpp>

#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

namespace NURBS
{

/// A B-spline curve \f$C(u) = \sum_{i=0}^{n} N_{i,p}(u) P_i\f$ of degree
/// \f$p\f$ (Eq. 3.1, *The NURBS Book*), defined by \f$n + 1\f$ control
/// points and a knot vector \f$U = \{u_0, \ldots, u_m\}\f$ with
/// \f$m = n + p + 1\f$.
///
/// The degree, knots and control points are stored by value. Evaluate() uses
/// CurvePoint() (Algorithm A3.1). The curve is defined on
/// \f$[u_p, u_{n+1}]\f$, which Domain() returns; for a clamped knot vector
/// this is \f$[u_0, u_m]\f$.
/// @tparam Point The control point type.
/// @tparam Scalar The floating-point type of the parameter and the knots, by
///         default ScalarOf<Point>.
/// @invariant There are at least \f$p + 1\f$ control points, there are
///            exactly \f$p + 1\f$ more knots than control points, and the
///            knots are non-decreasing.
template <typename Point, std::floating_point Scalar = ScalarOf<Point>>
    requires PointLike<Point, Scalar>
class BSplineCurve
{
public:
    /// Constructs a curve from its degree, knot vector and control points.
    /// @tparam RU An input range whose elements convert to @p Scalar.
    /// @tparam RP An input range whose elements convert to @p Point.
    /// @param degree The degree \f$p\f$.
    /// @param knots The knot vector \f$u_0, \ldots, u_m\f$.
    /// @param controlPoints The control points \f$P_0, \ldots, P_n\f$.
    /// @pre `controlPoints` has at least `degree + 1` elements, `knots` has
    ///      exactly `degree + 1` more, and the knots are non-decreasing.
    template <std::ranges::input_range RU, std::ranges::input_range RP>
        requires std::convertible_to<std::ranges::range_reference_t<RU>, Scalar> &&
                     std::convertible_to<std::ranges::range_reference_t<RP>, Point>
    constexpr BSplineCurve(std::size_t degree, const RU& knots, const RP& controlPoints)
        : m_degree(degree), m_knots(std::ranges::begin(knots), std::ranges::end(knots)),
          m_controlPoints(std::ranges::begin(controlPoints), std::ranges::end(controlPoints))
    {
        AssertValid();
    }

    /// Constructs a curve from its degree and lists of knots and control
    /// points.
    /// @param degree The degree \f$p\f$.
    /// @param knots The knot vector \f$u_0, \ldots, u_m\f$.
    /// @param controlPoints The control points \f$P_0, \ldots, P_n\f$.
    /// @pre `controlPoints` has at least `degree + 1` elements, `knots` has
    ///      exactly `degree + 1` more, and the knots are non-decreasing.
    constexpr BSplineCurve(std::size_t degree, std::initializer_list<Scalar> knots,
                           std::initializer_list<Point> controlPoints)
        : m_degree(degree), m_knots(knots), m_controlPoints(controlPoints)
    {
        AssertValid();
    }

    /// Computes a point on the curve with CurvePoint() (Algorithm A3.1).
    /// @param u The parameter value to evaluate at.
    /// @return The point \f$C(u)\f$.
    /// @pre @p u lies in Domain().
    [[nodiscard]] constexpr Point Evaluate(Scalar u) const { return CurvePoint(m_degree, m_knots, m_controlPoints, u); }

    /// Returns the degree of the curve.
    /// @return The degree \f$p\f$.
    [[nodiscard]] constexpr std::size_t Degree() const { return m_degree; }

    /// Returns the parameter domain of the curve.
    /// @return The pair \f$(u_p, u_{n+1})\f$.
    [[nodiscard]] constexpr std::pair<Scalar, Scalar> Domain() const
    {
        return {m_knots[m_degree], m_knots[m_controlPoints.size()]};
    }

    /// Returns the knot vector.
    ///
    /// The knots are read-only, since changing them could break the
    /// invariant; constructing a new curve changes them.
    /// @return A read-only view of \f$u_0, \ldots, u_m\f$.
    [[nodiscard]] constexpr std::span<const Scalar> Knots() const { return m_knots; }

    /// Returns the control points.
    /// @return A read-only view of \f$P_0, \ldots, P_n\f$.
    [[nodiscard]] constexpr std::span<const Point> ControlPoints() const { return m_controlPoints; }

    /// Returns the control points for editing in place.
    ///
    /// The number of control points is fixed, so changing it means
    /// constructing a new curve.
    /// @return A mutable view of \f$P_0, \ldots, P_n\f$.
    [[nodiscard]] constexpr std::span<Point> ControlPoints() { return m_controlPoints; }

private:
    constexpr void AssertValid() const
    {
        assert(m_controlPoints.size() >= m_degree + 1);
        assert(m_knots.size() == m_controlPoints.size() + m_degree + 1);
        assert(std::ranges::is_sorted(m_knots));
    }

    std::size_t m_degree;
    std::vector<Scalar> m_knots;
    std::vector<Point> m_controlPoints;
};

/// Deduces the point type of a BSplineCurve from a range of control points.
/// @tparam RU An input range of knots.
/// @tparam RP An input range of control points.
template <std::ranges::input_range RU, std::ranges::input_range RP>
BSplineCurve(std::size_t, const RU&, const RP&) -> BSplineCurve<std::ranges::range_value_t<RP>>;

} // namespace NURBS
