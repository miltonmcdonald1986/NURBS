/// @file
/// Umbrella header that includes the whole library.

#pragma once

#include <NURBS/bernstein.hpp>
#include <NURBS/bezier.hpp>
#include <NURBS/bezier_curve.hpp>
#include <NURBS/bezier_surface.hpp>
#include <NURBS/bspline.hpp>
#include <NURBS/bspline_curve.hpp>
#include <NURBS/bspline_basis.hpp>
#include <NURBS/point.hpp>
#include <NURBS/power_basis.hpp>
#include <NURBS/rational_bezier_curve.hpp>
#include <NURBS/rational_bezier_surface.hpp>

/// Algorithms from *The NURBS Book* (Piegl & Tiller).
///
/// Each algorithm is named after its number in the book (A1.1, A1.2, ...).
/// Curve and surface algorithms are templated on the point type, so any type that models
/// PointLike works, from plain scalars to vector types such as `glm::dvec3`.
namespace NURBS
{
}
