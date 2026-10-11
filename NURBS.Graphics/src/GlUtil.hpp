#pragma once

#include <glad/gl.h>

#include <cstddef>
#include <span>

namespace Graphics::Detail
{

// Enables float vertex attribute index with components floats at offset bytes
// into each stride-byte vertex of the bound GL_ARRAY_BUFFER.
inline void VertexAttribute(GLuint index, GLint components, GLsizei stride, std::size_t offset)
{
    glEnableVertexAttribArray(index);
    glVertexAttribPointer(index, components, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offset));
}

// glBufferData of data into the buffer bound to target.
template <typename T>
void UploadBuffer(GLenum target, std::span<const T> data, GLenum usage)
{
    glBufferData(target, static_cast<GLsizeiptr>(data.size_bytes()), data.data(), usage);
}

} // namespace Graphics::Detail
