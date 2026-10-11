/// @file
/// GlHandle, an owning handle to an OpenGL object, and its traits.

#pragma once

#include <glad/gl.h>

#include <utility>

namespace Graphics
{

/// Owns one OpenGL object name and deletes it with Traits::Delete when
/// destroyed or reset. 0 means empty. Move-only. The GL context must still be
/// current when a non-empty handle is destroyed.
///
/// Traits is a struct of static functions rather than GL functions because
/// glad's functions are pointers loaded at run time, which can't be template
/// arguments. Traits that also have a Create() (objects made by glGen*) enable
/// GlHandle::Create.
/// @tparam Traits A struct with a static `Delete(GLuint)` and, optionally, a
///                static `GLuint Create()`.
template <typename Traits>
class GlHandle
{
public:
    /// An empty handle.
    GlHandle() = default;
    /// Takes ownership of @p id.
    /// @param id The object name, or 0 for an empty handle.
    explicit GlHandle(GLuint id) : m_id(id) {}
    GlHandle(const GlHandle&) = delete;
    GlHandle& operator=(const GlHandle&) = delete;
    /// Takes ownership of @p other's object, leaving @p other empty.
    /// @param other The handle to move from.
    GlHandle(GlHandle&& other) noexcept : m_id(std::exchange(other.m_id, 0)) {}
    /// Deletes the current object, if any, and takes ownership of @p other's,
    /// leaving @p other empty.
    /// @param other The handle to move from.
    /// @return This handle.
    GlHandle& operator=(GlHandle&& other) noexcept
    {
        if (this != &other)
            Reset(std::exchange(other.m_id, 0));
        return *this;
    }
    /// Deletes the object, if any.
    ~GlHandle() { Reset(); }

    /// Creates a new object.
    /// @return A handle owning a new object from Traits::Create.
    [[nodiscard]] static GlHandle Create()
        requires requires { Traits::Create(); }
    {
        return GlHandle(Traits::Create());
    }

    /// The object name.
    /// @return The object name, or 0 if empty.
    [[nodiscard]] GLuint Get() const { return m_id; }
    /// Whether the handle owns an object.
    /// @return True if the handle is non-empty.
    [[nodiscard]] explicit operator bool() const { return m_id != 0; }

    /// Deletes the current object, if any, and takes ownership of @p id.
    /// @param id The new object name, or 0 to leave the handle empty.
    void Reset(GLuint id = 0)
    {
        if (m_id != 0)
            Traits::Delete(m_id);
        m_id = id;
    }

private:
    GLuint m_id = 0;
};

/// GlHandle traits for textures.
struct TextureTraits
{
    /// Creates a texture.
    /// @return The new texture's name.
    static GLuint Create()
    {
        GLuint id = 0;
        glGenTextures(1, &id);
        return id;
    }
    /// Deletes a texture.
    /// @param id The texture's name.
    static void Delete(GLuint id) { glDeleteTextures(1, &id); }
};

/// GlHandle traits for framebuffers.
struct FramebufferTraits
{
    /// Creates a framebuffer.
    /// @return The new framebuffer's name.
    static GLuint Create()
    {
        GLuint id = 0;
        glGenFramebuffers(1, &id);
        return id;
    }
    /// Deletes a framebuffer.
    /// @param id The framebuffer's name.
    static void Delete(GLuint id) { glDeleteFramebuffers(1, &id); }
};

/// GlHandle traits for renderbuffers.
struct RenderbufferTraits
{
    /// Creates a renderbuffer.
    /// @return The new renderbuffer's name.
    static GLuint Create()
    {
        GLuint id = 0;
        glGenRenderbuffers(1, &id);
        return id;
    }
    /// Deletes a renderbuffer.
    /// @param id The renderbuffer's name.
    static void Delete(GLuint id) { glDeleteRenderbuffers(1, &id); }
};

/// GlHandle traits for buffers.
struct BufferTraits
{
    /// Creates a buffer.
    /// @return The new buffer's name.
    static GLuint Create()
    {
        GLuint id = 0;
        glGenBuffers(1, &id);
        return id;
    }
    /// Deletes a buffer.
    /// @param id The buffer's name.
    static void Delete(GLuint id) { glDeleteBuffers(1, &id); }
};

/// GlHandle traits for vertex arrays.
struct VertexArrayTraits
{
    /// Creates a vertex array.
    /// @return The new vertex array's name.
    static GLuint Create()
    {
        GLuint id = 0;
        glGenVertexArrays(1, &id);
        return id;
    }
    /// Deletes a vertex array.
    /// @param id The vertex array's name.
    static void Delete(GLuint id) { glDeleteVertexArrays(1, &id); }
};

/// GlHandle traits for shaders. Shaders are created with
/// glCreateShader(type), so a GlShader is constructed from an id instead.
struct ShaderTraits
{
    /// Deletes a shader.
    /// @param id The shader's name.
    static void Delete(GLuint id) { glDeleteShader(id); }
};

/// GlHandle traits for programs. Programs are created with glCreateProgram,
/// so a GlProgram is constructed from an id instead.
struct ProgramTraits
{
    /// Deletes a program.
    /// @param id The program's name.
    static void Delete(GLuint id) { glDeleteProgram(id); }
};

/// An owned texture.
using GlTexture = GlHandle<TextureTraits>;
/// An owned framebuffer.
using GlFramebuffer = GlHandle<FramebufferTraits>;
/// An owned renderbuffer.
using GlRenderbuffer = GlHandle<RenderbufferTraits>;
/// An owned buffer.
using GlBuffer = GlHandle<BufferTraits>;
/// An owned vertex array.
using GlVertexArray = GlHandle<VertexArrayTraits>;
/// An owned shader.
using GlShader = GlHandle<ShaderTraits>;
/// An owned program.
using GlProgram = GlHandle<ProgramTraits>;

} // namespace Graphics
