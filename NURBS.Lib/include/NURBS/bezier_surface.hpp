/// @file
/// BezierSurface, a tensor-product Bezier surface.

#pragma once

#include <NURBS/bezier.hpp>
#include <NURBS/detail/grid.hpp>
#include <NURBS/point.hpp>

#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <ranges>

namespace NURBS
{

/// A tensor-product Bezier surface
/// \f$S(u, v) = \sum_{i=0}^{n} \sum_{j=0}^{m} B_{i,n}(u) B_{j,m}(v) P_{i,j}\f$
/// of degree \f$n\f$ in \f$u\f$ and \f$m\f$ in \f$v\f$, defined by an
/// \f$(n + 1) \times (m + 1)\f$ grid of control points.
///
/// The control points are stored by value. Evaluate() uses de Casteljau's
/// algorithm (deCasteljau2()).
/// @tparam Point The control point type.
/// @tparam Scalar The floating-point parameter type, by default
///         ScalarOf<Point>.
template <typename Point, std::floating_point Scalar = ScalarOf<Point>>
    requires PointLike<Point, Scalar>
class BezierSurface
{
public:
    /// Constructs a surface from a range of rows of control points.
    /// @tparam R An input range of rows, each an input range whose elements
    ///         convert to @p Point.
    /// @param controlPoints The control points, indexed `controlPoints[i][j]`
    ///        for \f$P_{i,j}\f$.
    /// @pre @p controlPoints is non-empty, its rows are non-empty, and all rows
    ///      have the same size.
    template <std::ranges::input_range R>
        requires std::ranges::input_range<std::ranges::range_reference_t<R>> &&
                 std::convertible_to<std::ranges::range_reference_t<std::ranges::range_reference_t<R>>, Point>
    explicit constexpr BezierSurface(const R& controlPoints)
        : m_controlPoints(controlPoints)
    {
    }

    /// Constructs a surface from a nested list of control points.
    /// @param controlPoints The rows of control points
    ///        \f$P_{i,0}, \ldots, P_{i,m}\f$, for \f$i = 0, \ldots, n\f$.
    /// @pre @p controlPoints is non-empty, its rows are non-empty, and all rows
    ///      have the same size.
    constexpr BezierSurface(std::initializer_list<std::initializer_list<Point>> controlPoints)
        : m_controlPoints(controlPoints)
    {
    }

    /// Computes a point on the surface with de Casteljau's algorithm.
    /// @param u The \f$u\f$ parameter value to evaluate at, normally in
    ///          \f$[0, 1]\f$.
    /// @param v The \f$v\f$ parameter value to evaluate at, normally in
    ///          \f$[0, 1]\f$.
    /// @return The point \f$S(u, v)\f$.
    [[nodiscard]] constexpr Point Evaluate(Scalar u, Scalar v) const
    {
        return deCasteljau2(m_controlPoints.Rows(), u, v);
    }

    /// Returns the degree in \f$u\f$.
    /// @return The degree \f$n\f$, one less than the number of rows.
    [[nodiscard]] constexpr std::size_t DegreeU() const { return m_controlPoints.RowCount() - 1; }

    /// Returns the degree in \f$v\f$.
    /// @return The degree \f$m\f$, one less than the number of columns.
    [[nodiscard]] constexpr std::size_t DegreeV() const { return m_controlPoints.ColumnCount() - 1; }

    /// Returns the control points.
    /// @return A random-access range of rows, each a read-only span, so that
    ///         `ControlPoints()[i][j]` is \f$P_{i,j}\f$.
    [[nodiscard]] constexpr auto ControlPoints() const { return m_controlPoints.Rows(); }

    /// Returns the control points for editing in place.
    ///
    /// The degrees are fixed, so changing them means constructing a new
    /// surface.
    /// @return A random-access range of rows, each a mutable span, so that
    ///         `ControlPoints()[i][j]` is \f$P_{i,j}\f$.
    [[nodiscard]] constexpr auto ControlPoints() { return m_controlPoints.Rows(); }

private:
    Detail::Grid<Point> m_controlPoints;
};

/// Deduces the point type of a BezierSurface from a range of rows of control
/// points.
/// @tparam R An input range of rows of control points.
template <std::ranges::input_range R>
BezierSurface(const R&) -> BezierSurface<std::ranges::range_value_t<std::ranges::range_value_t<R>>>;

} // namespace NURBS
