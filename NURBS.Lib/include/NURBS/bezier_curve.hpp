/// @file
/// BezierCurve, a polynomial Bezier curve.

#pragma once

#include <NURBS/bezier.hpp>
#include <NURBS/point.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <ranges>
#include <span>
#include <vector>

namespace NURBS
{

/// A Bezier curve \f$C(u) = \sum_{k=0}^{n} B_{k,n}(u) P_k\f$ of degree
/// \f$n\f$, defined by \f$n + 1\f$ control points.
///
/// The control points are stored by value. Evaluate() uses de Casteljau's
/// algorithm (deCasteljau1()).
/// @tparam Point The control point type.
/// @tparam Scalar The floating-point parameter type, by default
///         ScalarOf<Point>.
template <typename Point, std::floating_point Scalar = ScalarOf<Point>>
    requires PointLike<Point, Scalar>
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

} // namespace NURBS
