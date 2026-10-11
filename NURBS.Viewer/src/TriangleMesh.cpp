#include "TriangleMesh.hpp"

#include <cstddef>

namespace NURBS::Viewer
{

void TriangleMesh::Upload(const MeshData& data)
{
    if (!m_vertexArray)
    {
        GLuint id = 0;
        glGenVertexArrays(1, &id);
        m_vertexArray.Reset(id);
        glGenBuffers(1, &id);
        m_vertexBuffer.Reset(id);
        glGenBuffers(1, &id);
        m_indexBuffer.Reset(id);

        // The index buffer binding is vertex array state, so bind it while
        // the vertex array is bound.
        glBindVertexArray(m_vertexArray.Get());
        glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer.Get());
        constexpr auto kStride = static_cast<GLsizei>(sizeof(MeshVertex));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kStride,
                              reinterpret_cast<const void*>(offsetof(MeshVertex, position)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, kStride,
                              reinterpret_cast<const void*>(offsetof(MeshVertex, normal)));
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    // Dynamic, since a mesh is uploaded again whenever its source is edited.
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(data.vertices.size() * sizeof(MeshVertex)),
                 data.vertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(m_vertexArray.Get());
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(data.indices.size() * sizeof(std::uint32_t)),
                 data.indices.data(), GL_DYNAMIC_DRAW);
    glBindVertexArray(0);

    m_indexCount = static_cast<GLsizei>(data.indices.size());
}

void TriangleMesh::Draw() const
{
    if (IsEmpty())
        return;

    glBindVertexArray(m_vertexArray.Get());
    glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

} // namespace NURBS::Viewer
