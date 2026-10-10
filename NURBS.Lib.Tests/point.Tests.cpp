#include <gtest/gtest.h>

#include <NURBS/point.hpp>

#include <glm/glm.hpp>

#include <concepts>

namespace
{

// NoAdd is a dummy point type with scalar multiplication (double * NoAdd) but no operator+. It meets half of
// CurvePoint, so the concept should reject it. The operator just returns p because its body never runs; only its
// existence matters to the concept, hence [[maybe_unused]].
struct NoAdd
{
};

[[maybe_unused]] constexpr NoAdd operator*(double, const NoAdd& p) { return p; }

// NoScale is the opposite case: it has addition but no scalar * point. Together the two types show that the concept
// needs both operations, not just one.
struct NoScale
{
};

[[maybe_unused]] constexpr NoScale operator+(const NoScale& p, const NoScale&) { return p; }

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
