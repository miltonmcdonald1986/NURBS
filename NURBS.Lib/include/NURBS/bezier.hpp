#pragma once

#include <NURBS/bernstein.hpp>
#include <NURBS/horner1.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <ranges>
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

    const std::vector<Scalar> B = AllBernstein(n, u);
    Point C = B[0] * first[0];
    for (std::size_t k = 1; k <= n; ++k)
        C = C + B[k] * first[k];
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

private:
    std::vector<Point> m_controlPoints;
};

template <std::ranges::input_range R>
BezierCurve(const R&) -> BezierCurve<std::ranges::range_value_t<R>>;

} // namespace NURBS
