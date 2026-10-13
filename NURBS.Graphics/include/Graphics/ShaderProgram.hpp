/// @file
/// ShaderProgram, a linked OpenGL shader program.

#pragma once

#include "GlHandle.hpp"

#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace Graphics
{

/// A linked OpenGL shader program made of a vertex stage and a fragment stage.
///
/// GL 3.3 has no glProgramUniform, so the Set functions write to the program
/// in use: call Use() first.
class ShaderProgram
{
public:
    /// Compiles and links a program from a vertex and a fragment stage. Each
    /// stage's source is compiled after kGlslVersion, a newline, and
    /// kGlslPrecision.
    /// @param name The name printed with any error.
    /// @param vertexSource The vertex shader's GLSL source, without a
    ///                     `#version` directive.
    /// @param fragmentSource The fragment shader's GLSL source, without a
    ///                       `#version` directive.
    /// @return The program. On failure, @p name and the GL info log are printed
    ///         to stderr and the program is empty.
    [[nodiscard]] static ShaderProgram Build(const char* name, const char* vertexSource, const char* fragmentSource);

    /// Whether the program built.
    /// @return True if the program is non-empty.
    [[nodiscard]] explicit operator bool() const { return static_cast<bool>(m_program); }

    /// Makes this the program in use.
    void Use() const;

    /// Sets a `mat4` uniform of the program in use.
    /// @param uniform The uniform's name.
    /// @param value The value.
    void Set(const char* uniform, const glm::mat4& value) const;
    /// Sets a `mat3` uniform of the program in use.
    /// @param uniform The uniform's name.
    /// @param value The value.
    void Set(const char* uniform, const glm::mat3& value) const;
    /// Sets a `vec2` uniform of the program in use.
    /// @param uniform The uniform's name.
    /// @param value The value.
    void Set(const char* uniform, glm::vec2 value) const;
    /// Sets a `vec3` uniform of the program in use.
    /// @param uniform The uniform's name.
    /// @param value The value.
    void Set(const char* uniform, glm::vec3 value) const;
    /// Sets a `float` uniform of the program in use.
    /// @param uniform The uniform's name.
    /// @param value The value.
    void Set(const char* uniform, float value) const;

private:
    [[nodiscard]] GLint Location(const char* uniform) const;

    GlProgram m_program;
};

} // namespace Graphics
