#pragma once

// Point types that each meet only half of NURBS::PointLike, for checking that
// the concept, and everything constrained by it, rejects them.

namespace NURBS::Tests
{

// NoAdd is a dummy point type with scalar multiplication (double * NoAdd) but no operator+. It meets half of
// PointLike, so the concept should reject it. The operator just returns p because its body never runs; only its
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

} // namespace NURBS::Tests
