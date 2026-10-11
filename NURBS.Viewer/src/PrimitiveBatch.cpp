#include "PrimitiveBatch.hpp"

#include <cstddef>

namespace NURBS::Viewer
{

void PrimitiveBatch::Draw(GLenum mode)
{
    if (IsEmpty())
        return;

    if (!m_vertexArray)
    {
        GLuint id = 0;
        glGenVertexArrays(1, &id);
        m_vertexArray.Reset(id);
        glGenBuffers(1, &id);
        m_vertexBuffer.Reset(id);

        glBindVertexArray(m_vertexArray.Get());
        glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
        constexpr auto kStride = static_cast<GLsizei>(sizeof(PrimitiveVertex));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kStride,
                              reinterpret_cast<const void*>(offsetof(PrimitiveVertex, position)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, kStride,
                              reinterpret_cast<const void*>(offsetof(PrimitiveVertex, color)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, kStride,
                              reinterpret_cast<const void*>(offsetof(PrimitiveVertex, size)));
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    // Stream, since the batch is refilled every frame.
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m_vertices.size() * sizeof(PrimitiveVertex)),
                 m_vertices.data(), GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(m_vertexArray.Get());
    glDrawArrays(mode, 0, static_cast<GLsizei>(m_vertices.size()));
    glBindVertexArray(0);
}

} // namespace NURBS::Viewer
