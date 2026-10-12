/// @file
/// TriangleMesh, a triangle mesh on the GPU.

#pragma once

#include "GlHandle.hpp"
#include "MeshData.hpp"

namespace Graphics
{

/// An indexed triangle list on the GPU: a vertex array with a vertex buffer
/// (attribute 0 position, attribute 1 normal) and an index buffer.
///
/// GL objects are created on the first Upload, so the GL context must be
/// current then, and still current when the mesh is destroyed.
class TriangleMesh
{
public:
    /// Replaces the mesh's contents with @p data.
    /// @param data The triangles to upload.
    void Upload(const MeshData& data);
    /// Draws the triangles with the program in use. Does nothing if empty.
    void Draw() const;

    /// Whether there are no triangles.
    /// @return True if the mesh is empty.
    [[nodiscard]] bool IsEmpty() const { return m_indexCount == 0; }

private:
    void CreateVertexArray();

    GlVertexArray m_vertexArray;
    GlBuffer m_vertexBuffer;
    GlBuffer m_indexBuffer;
    GLsizei m_indexCount = 0;
};

} // namespace Graphics
