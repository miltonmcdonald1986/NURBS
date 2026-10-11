#pragma once

#include "TriangleMesh.hpp"

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace NURBS::Viewer
{

// Samples eval(u, v) on a grid of (uSegments + 1) x (vSegments + 1) evenly
// spaced parameters over [uMin, uMax] x [vMin, vMax] and triangulates it, two
// triangles per grid cell, wound counterclockwise about S_u x S_v.
//
// Normals come from differences of neighboring samples (central inside,
// one-sided on the edges), so any surface works. Where those vanish, as along
// a degree 0 direction or at a pole, the normal falls back to +z.
// Precondition: uSegments >= 1 and vSegments >= 1.
template <typename Eval>
    requires std::invocable<Eval&, double, double>
[[nodiscard]] MeshData SampleSurfaceMesh(Eval&& eval, double uMin, double uMax, double vMin, double vMax,
                                         std::size_t uSegments, std::size_t vSegments)
{
    assert(uSegments >= 1 && vSegments >= 1);

    const std::size_t uCount = uSegments + 1;
    const std::size_t vCount = vSegments + 1;
    const auto index = [vCount](std::size_t i, std::size_t j) { return i * vCount + j; };

    std::vector<glm::dvec3> points;
    points.reserve(uCount * vCount);
    for (std::size_t i = 0; i < uCount; ++i)
    {
        const double u = uMin + (uMax - uMin) * (static_cast<double>(i) / static_cast<double>(uSegments));
        for (std::size_t j = 0; j < vCount; ++j)
        {
            const double v = vMin + (vMax - vMin) * (static_cast<double>(j) / static_cast<double>(vSegments));
            points.push_back(glm::dvec3(eval(u, v)));
        }
    }

    MeshData mesh;
    mesh.vertices.reserve(points.size());
    for (std::size_t i = 0; i < uCount; ++i)
    {
        for (std::size_t j = 0; j < vCount; ++j)
        {
            const std::size_t iPrev = i > 0 ? i - 1 : i;
            const std::size_t iNext = i + 1 < uCount ? i + 1 : i;
            const std::size_t jPrev = j > 0 ? j - 1 : j;
            const std::size_t jNext = j + 1 < vCount ? j + 1 : j;
            const glm::dvec3 du = points[index(iNext, j)] - points[index(iPrev, j)];
            const glm::dvec3 dv = points[index(i, jNext)] - points[index(i, jPrev)];
            const glm::dvec3 cross = glm::cross(du, dv);

            // Relative to the step lengths, so the test doesn't depend on scale.
            const double length = glm::length(cross);
            const double scale = glm::length(du) * glm::length(dv);
            const glm::dvec3 normal = length > 1e-12 * scale && length > 0.0 ? cross / length : glm::dvec3(0.0, 0.0, 1.0);

            mesh.vertices.push_back({glm::vec3(points[index(i, j)]), glm::vec3(normal)});
        }
    }

    mesh.indices.reserve(6 * uSegments * vSegments);
    for (std::size_t i = 0; i < uSegments; ++i)
    {
        for (std::size_t j = 0; j < vSegments; ++j)
        {
            const auto a = static_cast<std::uint32_t>(index(i, j));
            const auto b = static_cast<std::uint32_t>(index(i + 1, j));
            const auto c = static_cast<std::uint32_t>(index(i + 1, j + 1));
            const auto d = static_cast<std::uint32_t>(index(i, j + 1));
            mesh.indices.insert(mesh.indices.end(), {a, b, c, a, c, d});
        }
    }
    return mesh;
}

} // namespace NURBS::Viewer
