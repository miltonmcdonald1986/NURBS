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

/// Vertices collected on the CPU over a frame and drawn in one call: a vertex
/// buffer with attribute 0 position, 1 color, and 2 size.
///
/// GL objects are created on the first Draw, so the GL context must be current
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

private:
    void CreateVertexArray();

    std::vector<PrimitiveVertex> m_vertices;
    GlVertexArray m_vertexArray;
    GlBuffer m_vertexBuffer;
};

} // namespace Graphics
