#include <Graphics/PrimitiveBatch.hpp>

#include "GlUtil.hpp"

#include <cassert>
#include <cstddef>
#include <span>

namespace Graphics
{

void PrimitiveBatch::CreateVertexArray()
{
    m_vertexArray = GlVertexArray::Create();

    glBindVertexArray(m_vertexArray.Get());
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    constexpr auto kStride = static_cast<GLsizei>(sizeof(PrimitiveVertex));
    Detail::VertexAttribute(0, 3, kStride, offsetof(PrimitiveVertex, position));
    Detail::VertexAttribute(1, 4, kStride, offsetof(PrimitiveVertex, color));
    Detail::VertexAttribute(2, 1, kStride, offsetof(PrimitiveVertex, size));
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PrimitiveBatch::CreateInstancedArray(int verticesPerInstance)
{
    m_instancedArray = GlVertexArray::Create();
    m_instancedStride = verticesPerInstance;

    glBindVertexArray(m_instancedArray.Get());
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    const auto stride = static_cast<GLsizei>(static_cast<std::size_t>(verticesPerInstance) * sizeof(PrimitiveVertex));
    for (int k = 0; k < verticesPerInstance; ++k)
    {
        const auto first = static_cast<GLuint>(3 * k);
        const std::size_t base = static_cast<std::size_t>(k) * sizeof(PrimitiveVertex);
        Detail::VertexAttribute(first, 3, stride, base + offsetof(PrimitiveVertex, position));
        Detail::VertexAttribute(first + 1, 4, stride, base + offsetof(PrimitiveVertex, color));
        Detail::VertexAttribute(first + 2, 1, stride, base + offsetof(PrimitiveVertex, size));
        for (GLuint i = first; i < first + 3; ++i)
            glVertexAttribDivisor(i, 1);
    }
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PrimitiveBatch::Upload()
{
    if (!m_vertexBuffer)
        m_vertexBuffer = GlBuffer::Create();

    // Stream, since the batch is refilled every frame.
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    Detail::UploadBuffer(GL_ARRAY_BUFFER, std::span<const PrimitiveVertex>(m_vertices), GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PrimitiveBatch::Draw(GLenum mode)
{
    if (IsEmpty())
        return;

    Upload();
    if (!m_vertexArray)
        CreateVertexArray();

    glBindVertexArray(m_vertexArray.Get());
    glDrawArrays(mode, 0, static_cast<GLsizei>(m_vertices.size()));
    glBindVertexArray(0);
}

void PrimitiveBatch::DrawInstanced(int verticesPerInstance)
{
    assert(verticesPerInstance > 0);
    assert(m_vertices.size() % static_cast<std::size_t>(verticesPerInstance) == 0);
    if (IsEmpty())
        return;

    Upload();
    if (!m_instancedArray || m_instancedStride != verticesPerInstance)
        CreateInstancedArray(verticesPerInstance);

    const auto instances = static_cast<GLsizei>(m_vertices.size() / static_cast<std::size_t>(verticesPerInstance));
    glBindVertexArray(m_instancedArray.Get());
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, instances);
    glBindVertexArray(0);
}

} // namespace Graphics
