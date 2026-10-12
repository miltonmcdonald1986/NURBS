/// @file
/// Bezier curve and surface evaluation (A1.4, A1.5, A1.7) and the
/// BezierCurve, RationalBezierCurve, BezierSurface and RationalBezierSurface
/// classes.

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
template <std::floating_point Scalar,
          std::ranges::random_access_range R,
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
template <std::floating_point Scalar,
          std::ranges::random_access_range R,
          typename Row = std::ranges::range_value_t<R>,
          typename Point = std::ranges::range_value_t<Row>>
    requires std::ranges::sized_range<R> && std::ranges::random_access_range<Row> &&
             std::ranges::sized_range<Row> && PointLike<Point, Scalar>
[[nodiscard]] constexpr Point deCasteljau2(const R& P, Scalar u, Scalar v)
{
    assert(!std::ranges::empty(P));
    assert(!std::ranges::empty(*std::ranges::begin(P)));
    assert(std::ranges::all_of(P, [&P](const auto& row) {
        return std::ranges::size(row) == std::ranges::size(*std::ranges::begin(P));
    }));

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
            const auto column = std::views::transform(P, [j](const auto& row) -> Point {
                return std::ranges::begin(row)[static_cast<Diff>(j)];
            });
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

namespace Detail
{

// Views a flat row-major array as a random-access range of rows, each a span of `columns` elements.
template <typename T>
[[nodiscard]] constexpr auto Rows(std::span<T> flat, std::size_t columns)
{
    return std::views::iota(std::size_t{0}, flat.size() / columns) |
           std::views::transform([flat, columns](std::size_t i) { return flat.subspan(i * columns, columns); });
}

// A rectangular grid of values stored flat in row-major order, so rows cannot differ in length.
template <typename T>
class Grid
{
public:
    // Copies a range of rows. Pre: there is at least one row, the rows are non-empty, and all have the same size.
    template <std::ranges::input_range R>
        requires std::ranges::input_range<std::ranges::range_reference_t<R>> &&
                 std::convertible_to<std::ranges::range_reference_t<std::ranges::range_reference_t<R>>, T>
    explicit constexpr Grid(const R& rows)
    {
        std::size_t rowCount = 0;
        for (const auto& row : rows)
        {
            const std::size_t before = m_values.size();
            for (const auto& value : row)
                m_values.push_back(static_cast<T>(value));

            const std::size_t length = m_values.size() - before;
            if (rowCount == 0)
                m_columns = length;
            assert(length == m_columns);
            ++rowCount;
        }
        assert(rowCount > 0);
        assert(m_columns > 0);
    }

    // A rows-by-columns grid with every element equal to value.
    constexpr Grid(std::size_t rows, std::size_t columns, const T& value)
        : m_values(rows * columns, value),
          m_columns(columns)
    {
        assert(rows > 0);
        assert(columns > 0);
    }

    [[nodiscard]] constexpr std::size_t RowCount() const { return m_values.size() / m_columns; }
    [[nodiscard]] constexpr std::size_t ColumnCount() const { return m_columns; }

    [[nodiscard]] constexpr std::span<const T> Values() const { return m_values; }

    [[nodiscard]] constexpr auto Rows() const { return Detail::Rows(std::span<const T>{m_values}, m_columns); }
    [[nodiscard]] constexpr auto Rows() { return Detail::Rows(std::span<T>{m_values}, m_columns); }

private:
    std::vector<T> m_values;
    std::size_t m_columns = 0;
};

} // namespace Detail

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
