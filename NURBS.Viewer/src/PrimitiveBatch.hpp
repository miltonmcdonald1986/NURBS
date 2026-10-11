#pragma once

#include "GlHandle.hpp"

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <vector>

namespace NURBS::Viewer
{

// One line end or point as the GPU sees it.
struct PrimitiveVertex
{
    glm::vec3 position;
    glm::vec4 color;
    // Line thickness or point radius, in framebuffer pixels.
    float size;
};

// Vertices collected on the CPU over a frame and drawn in one call: a vertex
// buffer with attribute 0 position, 1 color, and 2 size.
//
// GL objects are created on the first Draw, so the GL context must be current
// then, and still current when the batch is destroyed.
class PrimitiveBatch
{
public:
    void Add(const PrimitiveVertex& vertex) { m_vertices.push_back(vertex); }
    void Clear() { m_vertices.clear(); }
    [[nodiscard]] bool IsEmpty() const { return m_vertices.empty(); }

    // Uploads the vertices and draws them as mode (e.g. GL_LINES) with the
    // program in use. Does nothing if empty.
    void Draw(GLenum mode);

private:
    std::vector<PrimitiveVertex> m_vertices;
    GlVertexArray m_vertexArray;
    GlBuffer m_vertexBuffer;
};

} // namespace NURBS::Viewer
