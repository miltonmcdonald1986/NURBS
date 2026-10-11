#include <Graphics/SurfaceMesh.hpp>

#include <glm/geometric.hpp>

namespace Graphics
{

std::vector<glm::dvec3> GridNormals(std::span<const glm::dvec3> points, std::size_t uCount, std::size_t vCount)
{
    assert(points.size() == uCount * vCount);

    const auto at = [&points, vCount](std::size_t i, std::size_t j) { return points[i * vCount + j]; };

    std::vector<glm::dvec3> normals;
    normals.reserve(points.size());
    for (std::size_t i = 0; i < uCount; ++i)
    {
        for (std::size_t j = 0; j < vCount; ++j)
        {
            const std::size_t iPrev = i > 0 ? i - 1 : i;
            const std::size_t iNext = i + 1 < uCount ? i + 1 : i;
            const std::size_t jPrev = j > 0 ? j - 1 : j;
            const std::size_t jNext = j + 1 < vCount ? j + 1 : j;
            const glm::dvec3 du = at(iNext, j) - at(iPrev, j);
            const glm::dvec3 dv = at(i, jNext) - at(i, jPrev);
            const glm::dvec3 cross = glm::cross(du, dv);

            // Relative to the step lengths, so the test doesn't depend on scale.
            const double length = glm::length(cross);
            const double scale = glm::length(du) * glm::length(dv);
            normals.push_back(length > 1e-12 * scale ? cross / length : glm::dvec3(0.0, 0.0, 1.0));
        }
    }
    return normals;
}

std::vector<std::uint32_t> GridTriangleIndices(std::size_t uSegments, std::size_t vSegments)
{
    const std::size_t vCount = vSegments + 1;
    const auto index = [vCount](std::size_t i, std::size_t j) { return static_cast<std::uint32_t>(i * vCount + j); };

    std::vector<std::uint32_t> indices;
    indices.reserve(6 * uSegments * vSegments);
    for (std::size_t i = 0; i < uSegments; ++i)
    {
        for (std::size_t j = 0; j < vSegments; ++j)
        {
            const std::uint32_t a = index(i, j);
            const std::uint32_t b = index(i + 1, j);
            const std::uint32_t c = index(i + 1, j + 1);
            const std::uint32_t d = index(i, j + 1);
            indices.insert(indices.end(), {a, b, c, a, c, d});
        }
    }
    return indices;
}

std::vector<MeshVertex> MeshVertices(std::span<const glm::dvec3> points, std::span<const glm::dvec3> normals)
{
    assert(points.size() == normals.size());

    std::vector<MeshVertex> vertices;
    vertices.reserve(points.size());
    for (std::size_t k = 0; k < points.size(); ++k)
        vertices.push_back({glm::vec3(points[k]), glm::vec3(normals[k])});
    return vertices;
}

} // namespace Graphics
