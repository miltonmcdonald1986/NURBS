/// @file
/// MeshData, an indexed triangle mesh on the CPU.

#pragma once

#include <glm/vec3.hpp>

#include <cstdint>
#include <vector>

namespace Graphics
{

/// One mesh vertex as the GPU sees it (single precision).
struct MeshVertex
{
    /// The position.
    glm::vec3 position;
    /// The unit normal.
    glm::vec3 normal;
};

/// An indexed triangle list on the CPU, three indices per triangle.
struct MeshData
{
    /// The vertices.
    std::vector<MeshVertex> vertices;
    /// Indices into @ref vertices, three per triangle.
    std::vector<std::uint32_t> indices;
};

} // namespace Graphics
