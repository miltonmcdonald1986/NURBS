#include "TestPointTypes.hpp"

#include <gtest/gtest.h>

#include <NURBS/point.hpp>

#include <glm/glm.hpp>

#include <concepts>

namespace
{

using NURBS::Tests::NoAdd;
using NURBS::Tests::NoScale;

template <typename P>
concept HasScalarOf = requires { typename NURBS::ScalarOf<P>; };

} // namespace

TEST(CurvePoint, Constraints)
{
    static_assert(NURBS::CurvePoint<double, double>);
    static_assert(NURBS::CurvePoint<glm::dvec2, double>);
    static_assert(NURBS::CurvePoint<glm::dvec3, double>);
    static_assert(NURBS::CurvePoint<glm::vec3, float>);
    static_assert(!NURBS::CurvePoint<NoAdd, double>);
    static_assert(!NURBS::CurvePoint<NoScale, double>);
}

TEST(ScalarOf, Deduces)
{
    static_assert(std::same_as<NURBS::ScalarOf<double>, double>);
    static_assert(std::same_as<NURBS::ScalarOf<float>, float>);
    static_assert(std::same_as<NURBS::ScalarOf<glm::dvec3>, double>);
    static_assert(std::same_as<NURBS::ScalarOf<glm::vec2>, float>);
    static_assert(!HasScalarOf<int>);
    static_assert(!HasScalarOf<glm::ivec3>);
}
