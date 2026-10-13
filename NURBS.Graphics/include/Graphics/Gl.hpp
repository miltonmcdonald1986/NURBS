/// @file
/// The OpenGL API header. Include this instead of a GL loader or system header,
/// so the platform's choice is made in one place.

#pragma once

#ifdef __EMSCRIPTEN__
#include <GLES3/gl3.h>
#else
#include <glad/gl.h>
#endif

namespace Graphics
{

#ifdef __EMSCRIPTEN__
/// The GLSL `#version` directive for the GL context App creates. Shader
/// sources leave it out: ShaderProgram::Build prepends it, and App passes it
/// to ImGui's OpenGL backend.
inline constexpr const char* kGlslVersion = "#version 300 es";
/// The default precisions ShaderProgram::Build declares after kGlslVersion.
/// GLSL ES fragment shaders have no default float precision.
inline constexpr const char* kGlslPrecision = "precision highp float;\n";
#else
/// The GLSL `#version` directive for the GL context App creates. Shader
/// sources leave it out: ShaderProgram::Build prepends it, and App passes it
/// to ImGui's OpenGL backend.
inline constexpr const char* kGlslVersion = "#version 330 core";
/// The default precisions ShaderProgram::Build declares after kGlslVersion.
/// Desktop GLSL needs none.
inline constexpr const char* kGlslPrecision = "";
#endif

} // namespace Graphics
