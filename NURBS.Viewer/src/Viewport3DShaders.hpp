#pragma once

// GLSL sources for Viewport3D's programs.

namespace NURBS::Viewer::Shaders
{

// Lit meshes: positions and normals are passed on in view space, where the
// light sits at the eye (the origin).
inline constexpr const char* kMeshVertex = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

uniform mat4 uView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

out vec3 vPosition;
out vec3 vNormal;

void main()
{
    vec4 position = uView * vec4(aPosition, 1.0);
    vPosition = position.xyz;
    vNormal = uNormalMatrix * aNormal;
    gl_Position = uProjection * position;
}
)";

// Two-sided headlight shading: ambient, diffuse, and a mild Blinn-Phong
// highlight. The back side flips the normal and takes its own color.
inline constexpr const char* kMeshFragment = R"(#version 330 core
in vec3 vPosition;
in vec3 vNormal;

uniform vec3 uFrontColor;
uniform vec3 uBackColor;

out vec4 fragColor;

void main()
{
    vec3 n = normalize(gl_FrontFacing ? vNormal : -vNormal);
    vec3 color = gl_FrontFacing ? uFrontColor : uBackColor;
    vec3 toEye = normalize(-vPosition);
    float diffuse = max(dot(n, toEye), 0.0);
    // The light is at the eye, so the Blinn half vector is toEye itself.
    float specular = pow(diffuse, 32.0);
    fragColor = vec4(color * (0.25 + 0.75 * diffuse) + vec3(0.25 * specular), 1.0);
}
)";

// Lines and points: each vertex is moved toward the eye by uLift times its
// size in pixels, converted to world units at its depth. Moving along the eye
// ray leaves its screen position unchanged, but keeps a line or point lying on
// a surface from sinking into it.
inline constexpr const char* kPrimitiveVertex = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec4 aColor;
layout(location = 2) in float aSize;

uniform mat4 uView;
uniform mat4 uProjection;
uniform vec2 uViewportSize;
uniform float uLift;

out vec4 vColor;
out float vSize;

void main()
{
    vec4 position = uView * vec4(aPosition, 1.0);
    float w = (uProjection * position).w;
    // The world size of one pixel at this depth (w is 1 in orthographic).
    float pixel = 2.0 * w / (uProjection[1][1] * uViewportSize.y);
    bool perspective = uProjection[3][3] == 0.0;
    vec3 toEye = perspective && dot(position.xyz, position.xyz) > 0.0 ? normalize(-position.xyz) : vec3(0.0, 0.0, 1.0);
    position.xyz += toEye * (uLift * aSize * max(pixel, 0.0));
    gl_Position = uProjection * position;
    vColor = aColor;
    vSize = aSize;
}
)";

// Expands each segment into a quad of its thickness in pixels, extended by
// half the thickness at each end so the segments of a polyline overlap at the
// joints. Segments are clipped to the near plane first.
inline constexpr const char* kLineGeometry = R"(#version 330 core
layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec2 uViewportSize;

in vec4 vColor[];
in float vSize[];

out vec4 gColor;

void Emit(vec2 screen, vec4 clip, vec4 color)
{
    gl_Position = vec4(screen / (0.5 * uViewportSize) * clip.w, clip.z, clip.w);
    gColor = color;
    EmitVertex();
}

void main()
{
    vec4 a = gl_in[0].gl_Position;
    vec4 b = gl_in[1].gl_Position;
    float da = a.z + a.w;
    float db = b.z + b.w;
    if (da < 0.0 && db < 0.0)
        return;
    if (da < 0.0)
        a += (da / (da - db)) * (b - a);
    else if (db < 0.0)
        b += (db / (db - da)) * (a - b);

    vec2 sa = a.xy / a.w * 0.5 * uViewportSize;
    vec2 sb = b.xy / b.w * 0.5 * uViewportSize;
    vec2 d = sb - sa;
    float len = length(d);
    if (len < 1e-4)
        return;
    d /= len;
    vec2 n = vec2(-d.y, d.x);

    float ha = 0.5 * vSize[0];
    float hb = 0.5 * vSize[1];
    Emit(sa + ha * (n - d), a, vColor[0]);
    Emit(sa - ha * (n + d), a, vColor[0]);
    Emit(sb + hb * (n + d), b, vColor[1]);
    Emit(sb - hb * (n - d), b, vColor[1]);
    EndPrimitive();
}
)";

inline constexpr const char* kLineFragment = R"(#version 330 core
in vec4 gColor;

out vec4 fragColor;

void main()
{
    fragColor = gColor;
}
)";

// Expands each point into a square of twice its radius in pixels, with a
// local coordinate in [-1, 1]^2 for the fragment shader to cut a disc from.
inline constexpr const char* kPointGeometry = R"(#version 330 core
layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec2 uViewportSize;

in vec4 vColor[];
in float vSize[];

out vec4 gColor;
out vec2 gLocal;
out float gRadius;

void main()
{
    vec4 c = gl_in[0].gl_Position;
    if (c.w <= 0.0 || c.z + c.w < 0.0)
        return;

    vec2 offset = vSize[0] / (0.5 * uViewportSize) * c.w;
    for (int k = 0; k < 4; ++k)
    {
        vec2 corner = vec2((k & 1) == 0 ? -1.0 : 1.0, (k & 2) == 0 ? -1.0 : 1.0);
        gl_Position = vec4(c.xy + corner * offset, c.z, c.w);
        gColor = vColor[0];
        gLocal = corner;
        gRadius = vSize[0];
        EmitVertex();
    }
    EndPrimitive();
}
)";

// A disc whose edge fades out over about 1.5 pixels.
inline constexpr const char* kPointFragment = R"(#version 330 core
in vec4 gColor;
in vec2 gLocal;
in float gRadius;

out vec4 fragColor;

void main()
{
    float alpha = 1.0 - smoothstep(1.0 - 1.5 / max(gRadius, 1.0), 1.0, length(gLocal));
    if (alpha <= 0.0)
        discard;
    fragColor = vec4(gColor.rgb, gColor.a * alpha);
}
)";

} // namespace NURBS::Viewer::Shaders
