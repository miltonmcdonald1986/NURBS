#include <Graphics/PrimitiveBatch.hpp>

#include "GlUtil.hpp"

#include <cstddef>
#include <span>

namespace Graphics
{

void PrimitiveBatch::CreateVertexArray()
{
    m_vertexArray = GlVertexArray::Create();
    m_vertexBuffer = GlBuffer::Create();

    glBindVertexArray(m_vertexArray.Get());
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    constexpr auto kStride = static_cast<GLsizei>(sizeof(PrimitiveVertex));
    Detail::VertexAttribute(0, 3, kStride, offsetof(PrimitiveVertex, position));
    Detail::VertexAttribute(1, 4, kStride, offsetof(PrimitiveVertex, color));
    Detail::VertexAttribute(2, 1, kStride, offsetof(PrimitiveVertex, size));
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PrimitiveBatch::Draw(GLenum mode)
{
    if (IsEmpty())
        return;

    if (!m_vertexArray)
        CreateVertexArray();

    // Stream, since the batch is refilled every frame.
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    Detail::UploadBuffer(GL_ARRAY_BUFFER, std::span<const PrimitiveVertex>(m_vertices), GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(m_vertexArray.Get());
    glDrawArrays(mode, 0, static_cast<GLsizei>(m_vertices.size()));
    glBindVertexArray(0);
}

} // namespace Graphics
