#include "ShaderProgram.hpp"

#include <glm/gtc/type_ptr.hpp>

#include <cstdio>
#include <string>
#include <utility>

namespace NURBS::Viewer
{

namespace
{

// The info log of a shader or program, read with the matching GL functions.
template <typename GetIv, typename GetInfoLog>
[[nodiscard]] std::string InfoLog(GLuint id, GetIv getIv, GetInfoLog getInfoLog)
{
    GLint length = 0;
    getIv(id, GL_INFO_LOG_LENGTH, &length);
    std::string log(static_cast<std::size_t>(length > 0 ? length : 1), '\0');
    getInfoLog(id, static_cast<GLsizei>(log.size()), nullptr, log.data());
    return log;
}

// Compiles one stage, or returns an empty handle after logging the error.
[[nodiscard]] GlShader Compile(const char* name, const char* stage, GLenum type, const char* source)
{
    GlShader shader(glCreateShader(type));
    glShaderSource(shader.Get(), 1, &source, nullptr);
    glCompileShader(shader.Get());

    GLint ok = GL_FALSE;
    glGetShaderiv(shader.Get(), GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE)
    {
        std::fprintf(stderr, "%s: %s shader failed to compile:\n%s\n", name, stage,
                     InfoLog(shader.Get(), glGetShaderiv, glGetShaderInfoLog).c_str());
        return {};
    }
    return shader;
}

} // namespace

ShaderProgram ShaderProgram::Build(const char* name, const char* vertexSource, const char* fragmentSource)
{
    return Build(name, vertexSource, nullptr, fragmentSource);
}

ShaderProgram ShaderProgram::Build(const char* name, const char* vertexSource, const char* geometrySource,
                                   const char* fragmentSource)
{
    const GlShader vertex = Compile(name, "vertex", GL_VERTEX_SHADER, vertexSource);
    const GlShader geometry =
        geometrySource != nullptr ? Compile(name, "geometry", GL_GEOMETRY_SHADER, geometrySource) : GlShader{};
    const GlShader fragment = Compile(name, "fragment", GL_FRAGMENT_SHADER, fragmentSource);
    if (!vertex || (geometrySource != nullptr && !geometry) || !fragment)
        return {};

    GlProgram program(glCreateProgram());
    glAttachShader(program.Get(), vertex.Get());
    if (geometry)
        glAttachShader(program.Get(), geometry.Get());
    glAttachShader(program.Get(), fragment.Get());
    glLinkProgram(program.Get());
    // The shaders are deleted when they go out of scope; detaching lets GL
    // free them then, since the linked program no longer needs them.
    glDetachShader(program.Get(), vertex.Get());
    if (geometry)
        glDetachShader(program.Get(), geometry.Get());
    glDetachShader(program.Get(), fragment.Get());

    GLint ok = GL_FALSE;
    glGetProgramiv(program.Get(), GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE)
    {
        std::fprintf(stderr, "%s: program failed to link:\n%s\n", name,
                     InfoLog(program.Get(), glGetProgramiv, glGetProgramInfoLog).c_str());
        return {};
    }

    ShaderProgram result;
    result.m_program = std::move(program);
    return result;
}

void ShaderProgram::Use() const
{
    glUseProgram(m_program.Get());
}

GLint ShaderProgram::Location(const char* uniform) const
{
    return glGetUniformLocation(m_program.Get(), uniform);
}

void ShaderProgram::Set(const char* uniform, const glm::mat4& value) const
{
    glUniformMatrix4fv(Location(uniform), 1, GL_FALSE, glm::value_ptr(value));
}

void ShaderProgram::Set(const char* uniform, const glm::mat3& value) const
{
    glUniformMatrix3fv(Location(uniform), 1, GL_FALSE, glm::value_ptr(value));
}

void ShaderProgram::Set(const char* uniform, glm::vec2 value) const
{
    glUniform2fv(Location(uniform), 1, glm::value_ptr(value));
}

void ShaderProgram::Set(const char* uniform, glm::vec3 value) const
{
    glUniform3fv(Location(uniform), 1, glm::value_ptr(value));
}

void ShaderProgram::Set(const char* uniform, float value) const
{
    glUniform1f(Location(uniform), value);
}

} // namespace NURBS::Viewer
