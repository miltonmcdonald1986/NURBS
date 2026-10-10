/// @file
/// Umbrella header that includes the whole library.

#pragma once

#include <NURBS/bernstein.hpp>
#include <NURBS/bezier.hpp>
#include <NURBS/horner1.hpp>
#include <NURBS/horner2.hpp>

/// Algorithms from *The NURBS Book* (Piegl & Tiller).
///
/// Each algorithm is named after its number in the book (A1.1, A1.2, ...).
/// Curve and surface algorithms are templated on the point type, so any type that models
/// CurvePoint works, from plain scalars to vector types such as `glm::dvec3`.
namespace NURBS
{
}
