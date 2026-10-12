/// @file
/// PrimitiveBatch, which draws lines and points collected over a frame.

#pragma once

#include "GlHandle.hpp"

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <vector>

namespace Graphics
{

/// One line end or point as the GPU sees it.
struct PrimitiveVertex
{
    /// The position.
    glm::vec3 position;
    /// The RGBA color.
    glm::vec4 color;
    /// Line thickness or point radius, in framebuffer pixels.
    float size;
};

/// Vertices collected on the CPU over a frame and drawn in one call.
///
/// Draw reads one vertex per GL vertex: attribute 0 position, 1 color, and 2
/// size. DrawInstanced reads a run of vertices per instance instead, the k-th
/// of each run as attributes 3k (position), 3k + 1 (color), and 3k + 2 (size).
///
/// GL objects are created on the first draw, so the GL context must be current
/// then, and still current when the batch is destroyed.
class PrimitiveBatch
{
public:
    /// Appends a vertex.
    /// @param vertex The vertex.
    void Add(const PrimitiveVertex& vertex) { m_vertices.push_back(vertex); }
    /// Removes every vertex.
    void Clear() { m_vertices.clear(); }
    /// Whether there are no vertices.
    /// @return True if the batch is empty.
    [[nodiscard]] bool IsEmpty() const { return m_vertices.empty(); }

    /// Uploads the vertices and draws them as @p mode with the program in use.
    /// Does nothing if empty.
    /// @param mode The GL primitive mode, e.g. `GL_LINES`.
    void Draw(GLenum mode);
    /// Uploads the vertices and draws a 4-vertex `GL_TRIANGLE_STRIP` per
    /// instance with the program in use, which picks the corner from
    /// `gl_VertexID`. Does nothing if empty.
    /// @param verticesPerInstance The vertices each instance reads; the vertex
    ///     count must be a multiple of it.
    void DrawInstanced(int verticesPerInstance);

private:
    void CreateVertexArray();
    void CreateInstancedArray(int verticesPerInstance);
    // Streams the vertices into the vertex buffer, creating it if needed.
    void Upload();

    std::vector<PrimitiveVertex> m_vertices;
    GlBuffer m_vertexBuffer;
    GlVertexArray m_vertexArray;
    GlVertexArray m_instancedArray;
    // The verticesPerInstance m_instancedArray was laid out for.
    int m_instancedStride = 0;
};

} // namespace Graphics
