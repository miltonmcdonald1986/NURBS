/// @file
/// Point abstractions shared by the algorithms: the CurvePoint concept and
/// the ScalarOf trait.

#pragma once

#include <concepts>

namespace NURBS
{

/// A point type usable as a curve coefficient or control point: copyable,
/// closed under addition, and closed under multiplication by a scalar of
/// type @p S.
/// @tparam P The point type, e.g. `double` or `glm::dvec3`.
/// @tparam S The scalar type the point is multiplied by.
template <typename P, typename S>
concept CurvePoint = std::copyable<P> && requires(const P p, const P q, const S s) {
    { p + q } -> std::convertible_to<P>;
    { s * p } -> std::convertible_to<P>;
};

/// Implementation of ScalarOf. Only the specializations below are defined.
/// @tparam Point The point type to find the scalar type of.
template <typename Point>
struct ScalarOfImpl;

/// ScalarOfImpl for a floating-point number, which is its own scalar type.
/// @tparam Point A floating-point type.
template <std::floating_point Point>
struct ScalarOfImpl<Point>
{
    using type = Point; ///< The scalar type: @p Point itself.
};

/// ScalarOfImpl for a vector type with a floating-point `value_type`.
/// @tparam Point A type such as `glm::dvec3`.
template <typename Point>
    requires std::floating_point<typename Point::value_type>
struct ScalarOfImpl<Point>
{
    using type = typename Point::value_type; ///< The scalar type: `Point::value_type`.
};

/// The scalar type of a point: the point itself if it is a floating-point
/// number, otherwise its floating-point `value_type`
/// (e.g. `glm::dvec3` \f$\to\f$ `double`).
/// @tparam Point The point type.
template <typename Point>
using ScalarOf = typename ScalarOfImpl<Point>::type;

} // namespace NURBS
