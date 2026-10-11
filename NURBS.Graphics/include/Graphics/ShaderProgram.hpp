#pragma once

#include "GlHandle.hpp"

#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace Graphics
{

// A linked OpenGL shader program made of a vertex stage, an optional geometry
// stage, and a fragment stage.
//
// GL 3.3 has no glProgramUniform, so the Set functions write to the program
// in use: call Use() first.
class ShaderProgram
{
public:
    // Compiles and links the program. On failure, prints name and the GL info
    // log to stderr and returns an empty program.
    [[nodiscard]] static ShaderProgram Build(const char* name, const char* vertexSource, const char* fragmentSource);
    // As above, with a geometry stage between the two.
    [[nodiscard]] static ShaderProgram Build(const char* name, const char* vertexSource, const char* geometrySource,
                                             const char* fragmentSource);

    [[nodiscard]] explicit operator bool() const { return static_cast<bool>(m_program); }

    void Use() const;

    void Set(const char* uniform, const glm::mat4& value) const;
    void Set(const char* uniform, const glm::mat3& value) const;
    void Set(const char* uniform, glm::vec2 value) const;
    void Set(const char* uniform, glm::vec3 value) const;
    void Set(const char* uniform, float value) const;

private:
    [[nodiscard]] GLint Location(const char* uniform) const;

    GlProgram m_program;
};

} // namespace Graphics
