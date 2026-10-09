#pragma once

#include <NURBS/bernstein.hpp>
#include <NURBS/horner1.hpp>

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

// Algorithm A1.4 (The NURBS Book): compute the point C(u) = sum B_{k,n}(u) * P[k],
// k = 0..n, on a Bezier curve, using AllBernstein to compute the basis functions.
// The degree n is P.size() - 1. Precondition: P is non-empty.
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

// Algorithm A1.5 (The NURBS Book): compute the point C(u) on a Bezier curve
// using de Casteljau's algorithm, repeatedly applying the linear interpolation
// Q[i] = (1-u) * Q[i] + u * Q[i+1]. The degree n is P.size() - 1.
// Precondition: P is non-empty.
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

// The scalar type of a point: the point itself if it is a floating-point
// number, otherwise its floating-point value_type (e.g. glm::dvec3 -> double).
template <typename Point>
struct ScalarOfImpl;

template <std::floating_point Point>
struct ScalarOfImpl<Point>
{
    using type = Point;
};

template <typename Point>
    requires std::floating_point<typename Point::value_type>
struct ScalarOfImpl<Point>
{
    using type = typename Point::value_type;
};

template <typename Point>
using ScalarOf = typename ScalarOfImpl<Point>::type;

// A Bezier curve C(u) = sum B_{k,n}(u) * P[k], k = 0..n, of degree
// n = P.size() - 1. Evaluate uses de Casteljau's algorithm (deCasteljau1).
// Precondition: the control points are non-empty.
template <typename Point, std::floating_point Scalar = ScalarOf<Point>>
    requires CurvePoint<Point, Scalar>
class BezierCurve
{
public:
    template <std::ranges::input_range R>
        requires std::convertible_to<std::ranges::range_reference_t<R>, Point>
    explicit constexpr BezierCurve(const R& controlPoints)
        : m_controlPoints(std::ranges::begin(controlPoints), std::ranges::end(controlPoints))
    {
        assert(!m_controlPoints.empty());
    }

    constexpr BezierCurve(std::initializer_list<Point> controlPoints)
        : m_controlPoints(controlPoints)
    {
        assert(!m_controlPoints.empty());
    }

    [[nodiscard]] constexpr Point Evaluate(Scalar u) const { return deCasteljau1(m_controlPoints, u); }

    [[nodiscard]] constexpr std::size_t Degree() const { return m_controlPoints.size() - 1; }

    // The control points P[0..n]. The mutable overload edits them in place;
    // the degree is fixed, so changing it means constructing a new curve.
    [[nodiscard]] constexpr std::span<const Point> ControlPoints() const { return m_controlPoints; }
    [[nodiscard]] constexpr std::span<Point> ControlPoints() { return m_controlPoints; }

private:
    std::vector<Point> m_controlPoints;
};

template <std::ranges::input_range R>
BezierCurve(const R&) -> BezierCurve<std::ranges::range_value_t<R>>;

// A rational Bezier curve C(u) = sum B_{k,n}(u) * w[k] * P[k] / sum B_{k,n}(u) * w[k],
// k = 0..n, of degree n = P.size() - 1 (The NURBS Book, Eq. 4.1). It is a polynomial
// Bezier curve in homogeneous space, Pw[k] = (w[k] * P[k], w[k]), projected back by
// dividing by the last coordinate. Since de Casteljau works on each coordinate
// independently, Evaluate runs deCasteljau1 separately on the numerator points
// w[k] * P[k] and on the weights, so no homogeneous point type is needed.
// Preconditions: the control points are non-empty, there is one weight per control
// point, and every weight is positive.
template <typename Point, std::floating_point Scalar = ScalarOf<Point>>
    requires CurvePoint<Point, Scalar>
class RationalBezierCurve
{
public:
    // All weights 1: the curve is the same as BezierCurve on the same control points.
    template <std::ranges::input_range R>
        requires std::convertible_to<std::ranges::range_reference_t<R>, Point>
    explicit constexpr RationalBezierCurve(const R& controlPoints)
        : m_controlPoints(std::ranges::begin(controlPoints), std::ranges::end(controlPoints)),
          m_weights(m_controlPoints.size(), Scalar{1})
    {
        assert(!m_controlPoints.empty());
    }

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

    [[nodiscard]] constexpr std::size_t Degree() const { return m_controlPoints.size() - 1; }

    // The control points P[0..n] and weights w[0..n]. The mutable overloads edit them
    // in place; edited weights must stay positive. The degree is fixed, so changing it
    // means constructing a new curve.
    [[nodiscard]] constexpr std::span<const Point> ControlPoints() const { return m_controlPoints; }
    [[nodiscard]] constexpr std::span<Point> ControlPoints() { return m_controlPoints; }
    [[nodiscard]] constexpr std::span<const Scalar> Weights() const { return m_weights; }
    [[nodiscard]] constexpr std::span<Scalar> Weights() { return m_weights; }

private:
    std::vector<Point> m_controlPoints;
    std::vector<Scalar> m_weights;
};

template <std::ranges::input_range R>
RationalBezierCurve(const R&) -> RationalBezierCurve<std::ranges::range_value_t<R>>;

template <std::ranges::input_range R, std::ranges::input_range W>
RationalBezierCurve(const R&, const W&) -> RationalBezierCurve<std::ranges::range_value_t<R>>;

} // namespace NURBS
