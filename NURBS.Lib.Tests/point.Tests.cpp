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

TEST(PointLike, Constraints)
{
    static_assert(NURBS::PointLike<double, double>);
    static_assert(NURBS::PointLike<glm::dvec2, double>);
    static_assert(NURBS::PointLike<glm::dvec3, double>);
    static_assert(NURBS::PointLike<glm::vec3, float>);
    static_assert(!NURBS::PointLike<NoAdd, double>);
    static_assert(!NURBS::PointLike<NoScale, double>);
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
