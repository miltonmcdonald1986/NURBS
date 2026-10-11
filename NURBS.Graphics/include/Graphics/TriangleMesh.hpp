#pragma once

#include "GlHandle.hpp"

#include <glm/vec3.hpp>

#include <cstdint>
#include <vector>

namespace Graphics
{

// One mesh vertex as the GPU sees it (single precision).
struct MeshVertex
{
    glm::vec3 position;
    glm::vec3 normal;
};

// An indexed triangle list on the CPU, three indices per triangle.
struct MeshData
{
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices;
};

// An indexed triangle list on the GPU: a vertex array with a vertex buffer
// (attribute 0 position, attribute 1 normal) and an index buffer.
//
// GL objects are created on the first Upload, so the GL context must be
// current then, and still current when the mesh is destroyed.
class TriangleMesh
{
public:
    // Replaces the mesh's contents with data.
    void Upload(const MeshData& data);
    // Draws the triangles with the program in use. Does nothing if empty.
    void Draw() const;

    [[nodiscard]] bool IsEmpty() const { return m_indexCount == 0; }

private:
    void CreateVertexArray();

    GlVertexArray m_vertexArray;
    GlBuffer m_vertexBuffer;
    GlBuffer m_indexBuffer;
    GLsizei m_indexCount = 0;
};

} // namespace Graphics
