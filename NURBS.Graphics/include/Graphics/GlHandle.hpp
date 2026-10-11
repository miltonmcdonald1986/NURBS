#pragma once

#include <glad/gl.h>

#include <utility>

namespace Graphics
{

// Owns one OpenGL object name and deletes it with Traits::Delete when
// destroyed or reset. 0 means empty. Move-only. The GL context must still be
// current when a non-empty handle is destroyed.
//
// Traits is a struct of static functions rather than GL functions because
// glad's functions are pointers loaded at run time, which can't be template
// arguments. Traits that also have a Create() (objects made by glGen*) enable
// GlHandle::Create.
template <typename Traits>
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

    // A new object from Traits::Create.
    [[nodiscard]] static GlHandle Create()
        requires requires { Traits::Create(); }
    {
        return GlHandle(Traits::Create());
    }

    [[nodiscard]] GLuint Get() const { return m_id; }
    [[nodiscard]] explicit operator bool() const { return m_id != 0; }

    // Deletes the current object, if any, and takes ownership of id.
    void Reset(GLuint id = 0)
    {
        if (m_id != 0)
            Traits::Delete(m_id);
        m_id = id;
    }

private:
    GLuint m_id = 0;
};

struct TextureTraits
{
    static GLuint Create()
    {
        GLuint id = 0;
        glGenTextures(1, &id);
        return id;
    }
    static void Delete(GLuint id) { glDeleteTextures(1, &id); }
};

struct FramebufferTraits
{
    static GLuint Create()
    {
        GLuint id = 0;
        glGenFramebuffers(1, &id);
        return id;
    }
    static void Delete(GLuint id) { glDeleteFramebuffers(1, &id); }
};

struct RenderbufferTraits
{
    static GLuint Create()
    {
        GLuint id = 0;
        glGenRenderbuffers(1, &id);
        return id;
    }
    static void Delete(GLuint id) { glDeleteRenderbuffers(1, &id); }
};

struct BufferTraits
{
    static GLuint Create()
    {
        GLuint id = 0;
        glGenBuffers(1, &id);
        return id;
    }
    static void Delete(GLuint id) { glDeleteBuffers(1, &id); }
};

struct VertexArrayTraits
{
    static GLuint Create()
    {
        GLuint id = 0;
        glGenVertexArrays(1, &id);
        return id;
    }
    static void Delete(GLuint id) { glDeleteVertexArrays(1, &id); }
};

// Shaders and programs are created with glCreateShader(type) and
// glCreateProgram, so they are constructed from an id instead.
struct ShaderTraits
{
    static void Delete(GLuint id) { glDeleteShader(id); }
};

struct ProgramTraits
{
    static void Delete(GLuint id) { glDeleteProgram(id); }
};

using GlTexture = GlHandle<TextureTraits>;
using GlFramebuffer = GlHandle<FramebufferTraits>;
using GlRenderbuffer = GlHandle<RenderbufferTraits>;
using GlBuffer = GlHandle<BufferTraits>;
using GlVertexArray = GlHandle<VertexArrayTraits>;
using GlShader = GlHandle<ShaderTraits>;
using GlProgram = GlHandle<ProgramTraits>;

} // namespace Graphics
