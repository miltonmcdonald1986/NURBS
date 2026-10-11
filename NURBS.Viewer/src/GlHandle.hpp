#pragma once

#include <glad/gl.h>

#include <utility>

namespace NURBS::Viewer
{

// Owns one OpenGL object name and deletes it with Deleter when destroyed or
// reset. 0 means empty. Move-only. The GL context must still be current when
// a non-empty handle is destroyed.
//
// Deleter is a functor rather than a GL function because glad's functions are
// pointers loaded at run time, which can't be template arguments.
template <typename Deleter>
class GlHandle
{
public:
    GlHandle() = default;
    explicit GlHandle(GLuint id) : m_id(id) {}
    GlHandle(const GlHandle&) = delete;
    GlHandle& operator=(const GlHandle&) = delete;
    GlHandle(GlHandle&& other) noexcept : m_id(std::exchange(other.m_id, 0)) {}
    GlHandle& operator=(GlHandle&& other) noexcept
    {
        if (this != &other)
            Reset(std::exchange(other.m_id, 0));
        return *this;
    }
    ~GlHandle() { Reset(); }

    [[nodiscard]] GLuint Get() const { return m_id; }
    [[nodiscard]] explicit operator bool() const { return m_id != 0; }

    // Deletes the current object, if any, and takes ownership of id.
    void Reset(GLuint id = 0)
    {
        if (m_id != 0)
            Deleter{}(m_id);
        m_id = id;
    }

private:
    GLuint m_id = 0;
};

struct TextureDeleter
{
    void operator()(GLuint id) const { glDeleteTextures(1, &id); }
};

struct FramebufferDeleter
{
    void operator()(GLuint id) const { glDeleteFramebuffers(1, &id); }
};

struct RenderbufferDeleter
{
    void operator()(GLuint id) const { glDeleteRenderbuffers(1, &id); }
};

struct BufferDeleter
{
    void operator()(GLuint id) const { glDeleteBuffers(1, &id); }
};

struct VertexArrayDeleter
{
    void operator()(GLuint id) const { glDeleteVertexArrays(1, &id); }
};

struct ShaderDeleter
{
    void operator()(GLuint id) const { glDeleteShader(id); }
};

struct ProgramDeleter
{
    void operator()(GLuint id) const { glDeleteProgram(id); }
};

using GlTexture = GlHandle<TextureDeleter>;
using GlFramebuffer = GlHandle<FramebufferDeleter>;
using GlRenderbuffer = GlHandle<RenderbufferDeleter>;
using GlBuffer = GlHandle<BufferDeleter>;
using GlVertexArray = GlHandle<VertexArrayDeleter>;
using GlShader = GlHandle<ShaderDeleter>;
using GlProgram = GlHandle<ProgramDeleter>;

} // namespace NURBS::Viewer
