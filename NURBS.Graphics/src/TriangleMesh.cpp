#include <Graphics/TriangleMesh.hpp>

#include "GlUtil.hpp"

#include <cstddef>
#include <span>

namespace Graphics
{

void TriangleMesh::CreateVertexArray()
{
    m_vertexArray = GlVertexArray::Create();
    m_vertexBuffer = GlBuffer::Create();
    m_indexBuffer = GlBuffer::Create();

    // The index buffer binding is vertex array state, so bind it while the
    // vertex array is bound.
    glBindVertexArray(m_vertexArray.Get());
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer.Get());
    constexpr auto kStride = static_cast<GLsizei>(sizeof(MeshVertex));
    Detail::VertexAttribute(0, 3, kStride, offsetof(MeshVertex, position));
    Detail::VertexAttribute(1, 3, kStride, offsetof(MeshVertex, normal));
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void TriangleMesh::Upload(const MeshData& data)
{
    if (!m_vertexArray)
        CreateVertexArray();

    // Dynamic, since a mesh is uploaded again whenever its source is edited.
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer.Get());
    Detail::UploadBuffer(GL_ARRAY_BUFFER, std::span<const MeshVertex>(data.vertices), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(m_vertexArray.Get());
    Detail::UploadBuffer(GL_ELEMENT_ARRAY_BUFFER, std::span<const std::uint32_t>(data.indices), GL_DYNAMIC_DRAW);
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

} // namespace Graphics
