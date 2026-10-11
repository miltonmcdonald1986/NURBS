#pragma once

#include "Sampling.hpp"
#include "TriangleMesh.hpp"

#include <glm/vec3.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace Graphics
{

// eval(u, v) on a grid of uCount x vCount evenly spaced parameters over
// [uMin, uMax] x [vMin, vMax], row by row: the point for the i-th u and j-th v
// is at index i * vCount + j. Precondition: uCount >= 2 and vCount >= 2.
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

// Unit normals of a grid laid out as by SampleGrid, along S_u x S_v, from
// differences of neighboring points (central inside, one-sided on the edges).
// Where those vanish, as along a degree 0 direction or at a pole, the normal
// falls back to +z. Precondition: points.size() == uCount * vCount.
[[nodiscard]] std::vector<glm::dvec3> GridNormals(std::span<const glm::dvec3> points, std::size_t uCount,
                                                  std::size_t vCount);

// Two triangles per cell of a grid with (uSegments + 1) x (vSegments + 1)
// points laid out as by SampleGrid, wound counterclockwise about S_u x S_v.
[[nodiscard]] std::vector<std::uint32_t> GridTriangleIndices(std::size_t uSegments, std::size_t vSegments);

// A grid's points and normals as single-precision mesh vertices.
// Precondition: points.size() == normals.size().
[[nodiscard]] std::vector<MeshVertex> MeshVertices(std::span<const glm::dvec3> points,
                                                   std::span<const glm::dvec3> normals);

// Samples eval(u, v) on a grid of (uSegments + 1) x (vSegments + 1) evenly
// spaced parameters over [uMin, uMax] x [vMin, vMax] and triangulates it, two
// triangles per grid cell, with normals from GridNormals, so any surface works.
// Precondition: uSegments >= 1 and vSegments >= 1.
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
