/// @file
/// The OpenGL API header. Include this instead of a GL loader or system header,
/// so the platform's choice is made in one place.

#pragma once

#include <glad/gl.h>

namespace Graphics
{

/// The GLSL `#version` directive for the GL context App creates. Shader
/// sources leave it out: ShaderProgram::Build prepends it, and App passes it
/// to ImGui's OpenGL backend.
inline constexpr const char* kGlslVersion = "#version 330 core";

} // namespace Graphics
