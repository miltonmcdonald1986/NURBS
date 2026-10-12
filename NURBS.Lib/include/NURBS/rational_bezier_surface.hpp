/// @file
/// RationalBezierSurface, a rational tensor-product Bezier surface (Eq. 4.15).

#pragma once

#include <NURBS/bezier.hpp>
#include <NURBS/detail/grid.hpp>
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

/// A rational tensor-product Bezier surface
/// \f$S(u, v) = \frac{\sum_{i=0}^{n} \sum_{j=0}^{m} B_{i,n}(u) B_{j,m}(v) w_{i,j} P_{i,j}}
///                    {\sum_{i=0}^{n} \sum_{j=0}^{m} B_{i,n}(u) B_{j,m}(v) w_{i,j}}\f$
/// of degree \f$n\f$ in \f$u\f$ and \f$m\f$ in \f$v\f$ (Eq. 4.15,
/// *The NURBS Book*).
///
/// It is a polynomial Bezier surface in homogeneous space,
/// \f$P^w_{i,j} = (w_{i,j} P_{i,j}, w_{i,j})\f$, projected back by dividing by
/// the last coordinate. As in RationalBezierCurve, Evaluate() runs
/// deCasteljau2() separately on the weighted points \f$w_{i,j} P_{i,j}\f$ and
/// on the weights, so no homogeneous point type is needed.
/// @tparam Point The control point type.
/// @tparam Scalar The floating-point type of the parameters and the weights,
///         by default ScalarOf<Point>.
/// @invariant There is one weight per control point, and every weight is
///            positive.
template <typename Point, std::floating_point Scalar = ScalarOf<Point>>
    requires PointLike<Point, Scalar>
class RationalBezierSurface
{
public:
    /// Constructs a surface with all weights 1, which is the same surface as a
    /// BezierSurface on the same control points.
    /// @tparam R An input range of rows, each an input range whose elements
    ///         convert to @p Point.
    /// @param controlPoints The control points, indexed `controlPoints[i][j]`
    ///        for \f$P_{i,j}\f$.
    /// @pre @p controlPoints is non-empty, its rows are non-empty, and all rows
    ///      have the same size.
    template <std::ranges::input_range R>
        requires std::ranges::input_range<std::ranges::range_reference_t<R>> &&
                 std::convertible_to<std::ranges::range_reference_t<std::ranges::range_reference_t<R>>, Point>
    explicit constexpr RationalBezierSurface(const R& controlPoints)
        : m_controlPoints(controlPoints),
          m_weights(m_controlPoints.RowCount(), m_controlPoints.ColumnCount(), Scalar{1})
    {
    }

    /// Constructs a surface from control points and their weights.
    /// @tparam R An input range of rows, each an input range whose elements
    ///         convert to @p Point.
    /// @tparam W An input range of rows, each an input range whose elements
    ///         convert to @p Scalar.
    /// @param controlPoints The control points, indexed `controlPoints[i][j]`
    ///        for \f$P_{i,j}\f$.
    /// @param weights The weights, indexed `weights[i][j]` for \f$w_{i,j}\f$.
    /// @pre @p controlPoints is non-empty, its rows are non-empty, and all rows
    ///      have the same size; @p weights has the same shape; and every weight
    ///      is positive.
    template <std::ranges::input_range R, std::ranges::input_range W>
        requires std::ranges::input_range<std::ranges::range_reference_t<R>> &&
                 std::convertible_to<std::ranges::range_reference_t<std::ranges::range_reference_t<R>>, Point> &&
                 std::ranges::input_range<std::ranges::range_reference_t<W>> &&
                 std::convertible_to<std::ranges::range_reference_t<std::ranges::range_reference_t<W>>, Scalar>
    constexpr RationalBezierSurface(const R& controlPoints, const W& weights)
        : m_controlPoints(controlPoints),
          m_weights(weights)
    {
        assert(m_weights.RowCount() == m_controlPoints.RowCount());
        assert(m_weights.ColumnCount() == m_controlPoints.ColumnCount());
        assert(std::ranges::all_of(m_weights.Values(), [](Scalar w) { return w > Scalar{0}; }));
    }

    /// Computes a point on the surface with de Casteljau's algorithm in
    /// homogeneous space.
    /// @param u The \f$u\f$ parameter value to evaluate at, normally in
    ///          \f$[0, 1]\f$.
    /// @param v The \f$v\f$ parameter value to evaluate at, normally in
    ///          \f$[0, 1]\f$.
    /// @return The point \f$S(u, v)\f$.
    [[nodiscard]] constexpr Point Evaluate(Scalar u, Scalar v) const
    {
        // The first coordinates of the homogeneous points Pw[i][j]: w[i][j] * P[i][j], flat in row-major order.
        std::vector<Point> weightedPoints;
        weightedPoints.reserve(m_controlPoints.Values().size());
        std::ranges::transform(m_weights.Values(), m_controlPoints.Values(), std::back_inserter(weightedPoints),
                               [](Scalar w, const Point& p) -> Point { return w * p; });

        // Evaluate both parts of Sw(u, v), then project back by dividing by its weight.
        const Point numerator =
            deCasteljau2(Detail::Rows(std::span<const Point>{weightedPoints}, m_controlPoints.ColumnCount()), u, v);
        const Scalar denominator = deCasteljau2(m_weights.Rows(), u, v);
        return (Scalar{1} / denominator) * numerator;
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

    /// Returns the weights.
    /// @return A random-access range of rows, each a read-only span, so that
    ///         `Weights()[i][j]` is \f$w_{i,j}\f$.
    [[nodiscard]] constexpr auto Weights() const { return m_weights.Rows(); }

    /// Returns the weights for editing in place.
    ///
    /// Edited weights must stay positive.
    /// @return A random-access range of rows, each a mutable span, so that
    ///         `Weights()[i][j]` is \f$w_{i,j}\f$.
    [[nodiscard]] constexpr auto Weights() { return m_weights.Rows(); }

private:
    Detail::Grid<Point> m_controlPoints;
    Detail::Grid<Scalar> m_weights;
};

/// Deduces the point type of a RationalBezierSurface from a range of rows of
/// control points.
/// @tparam R An input range of rows of control points.
template <std::ranges::input_range R>
RationalBezierSurface(const R&) -> RationalBezierSurface<std::ranges::range_value_t<std::ranges::range_value_t<R>>>;

/// Deduces the point type of a RationalBezierSurface from a range of rows of
/// control points and a range of rows of weights.
/// @tparam R An input range of rows of control points.
/// @tparam W An input range of rows of weights.
template <std::ranges::input_range R, std::ranges::input_range W>
RationalBezierSurface(const R&, const W&)
    -> RationalBezierSurface<std::ranges::range_value_t<std::ranges::range_value_t<R>>>;

} // namespace NURBS
