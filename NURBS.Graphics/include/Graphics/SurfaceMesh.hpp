/// @file
/// Sampling and triangulating parametric surfaces into meshes.

#pragma once

#include "MeshData.hpp"
#include "Sampling.hpp"

#include <glm/vec3.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace Graphics
{

/// Evaluates @p eval on a grid of @p uCount x @p vCount evenly spaced
/// parameters over [@p uMin, @p uMax] x [@p vMin, @p vMax], row by row: the
/// point for the i-th u and j-th v is at index `i * vCount + j`.
/// @tparam Eval A callable taking `(double u, double v)` and returning a point
///              convertible to `glm::dvec3`.
/// @param eval The surface, as a function of its parameters.
/// @param uMin The first u parameter.
/// @param uMax The last u parameter.
/// @param vMin The first v parameter.
/// @param vMax The last v parameter.
/// @param uCount The number of u parameters.
/// @param vCount The number of v parameters.
/// @return The grid's points.
/// @pre @p uCount >= 2 and @p vCount >= 2.
template <typename Eval>
    requires std::invocable<Eval&, double, double>
[[nodiscard]] std::vector<glm::dvec3> SampleGrid(Eval&& eval, double uMin, double uMax, double vMin, double vMax,
                                                 std::size_t uCount, std::size_t vCount)
{
    std::vector<glm::dvec3> points;
    points.reserve(uCount * vCount);
    for (std::size_t i = 0; i < uCount; ++i)
    {
        const double u = ParameterAt(uMin, uMax, i, uCount);
        for (std::size_t j = 0; j < vCount; ++j)
        {
            points.push_back(glm::dvec3(eval(u, ParameterAt(vMin, vMax, j, vCount))));
        }
    }
    return points;
}

/// Unit normals of a grid laid out as by SampleGrid, along
/// \f$S_u \times S_v\f$, from differences of neighboring points (central
/// inside, one-sided on the edges). Where those vanish, as along a degree 0
/// direction or at a pole, the normal falls back to +z.
/// @param points The grid's points.
/// @param uCount The number of u parameters.
/// @param vCount The number of v parameters.
/// @return One normal per point, in the same order.
/// @pre `points.size() == uCount * vCount`.
[[nodiscard]] std::vector<glm::dvec3> GridNormals(std::span<const glm::dvec3> points, std::size_t uCount,
                                                  std::size_t vCount);

/// Two triangles per cell of a grid with (@p uSegments + 1) x (@p vSegments + 1)
/// points laid out as by SampleGrid, wound counterclockwise about \f$S_u \times S_v\f$.
/// @param uSegments The number of cells in u.
/// @param vSegments The number of cells in v.
/// @return Three point indices per triangle.
[[nodiscard]] std::vector<std::uint32_t> GridTriangleIndices(std::size_t uSegments, std::size_t vSegments);

/// A grid's points and normals as single-precision mesh vertices.
/// @param points The points.
/// @param normals The normals, one per point.
/// @return One vertex per point, in the same order.
/// @pre `points.size() == normals.size()`.
[[nodiscard]] std::vector<MeshVertex> MeshVertices(std::span<const glm::dvec3> points,
                                                   std::span<const glm::dvec3> normals);

/// Samples @p eval on a grid of (@p uSegments + 1) x (@p vSegments + 1) evenly
/// spaced parameters over [@p uMin, @p uMax] x [@p vMin, @p vMax] and
/// triangulates it, two triangles per grid cell, with normals from
/// GridNormals, so any surface works.
/// @tparam Eval A callable taking `(double u, double v)` and returning a point
///              convertible to `glm::dvec3`.
/// @param eval The surface, as a function of its parameters.
/// @param uMin The first u parameter.
/// @param uMax The last u parameter.
/// @param vMin The first v parameter.
/// @param vMax The last v parameter.
/// @param uSegments The number of cells in u.
/// @param vSegments The number of cells in v.
/// @return The triangle mesh.
/// @pre @p uSegments >= 1 and @p vSegments >= 1.
template <typename Eval>
    requires std::invocable<Eval&, double, double>
[[nodiscard]] MeshData SampleSurfaceMesh(Eval&& eval, double uMin, double uMax, double vMin, double vMax,
                                         std::size_t uSegments, std::size_t vSegments)
{
    assert(uSegments >= 1 && vSegments >= 1);

    const std::size_t uCount = uSegments + 1;
    const std::size_t vCount = vSegments + 1;
    const std::vector<glm::dvec3> points = SampleGrid(eval, uMin, uMax, vMin, vMax, uCount, vCount);
    return {MeshVertices(points, GridNormals(points, uCount, vCount)), GridTriangleIndices(uSegments, vSegments)};
}

} // namespace Graphics
