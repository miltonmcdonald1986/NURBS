#pragma once

// GLSL sources for Viewport3D's programs, without the #version directive that
// ShaderProgram::Build prepends.

namespace Graphics::Shaders
{

// Lit meshes: positions and normals are passed on in view space, where the
// light sits at the eye (the origin).
inline constexpr const char* kMeshVertex = R"(
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
inline constexpr const char* kMeshFragment = R"(
in vec3 vPosition;
in vec3 vNormal;

uniform vec3 uFrontColor;
uniform vec3 uBackColor;
uniform float uOpacity;

out vec4 fragColor;

void main()
{
    vec3 n = normalize(gl_FrontFacing ? vNormal : -vNormal);
    vec3 color = gl_FrontFacing ? uFrontColor : uBackColor;
    vec3 toEye = normalize(-vPosition);
    float diffuse = max(dot(n, toEye), 0.0);
    // The light is at the eye, so the Blinn half vector is toEye itself.
    float specular = pow(diffuse, 32.0);
    fragColor = vec4(color * (0.25 + 0.75 * diffuse) + vec3(0.25 * specular), uOpacity);
}
)";

// Lines, one instance per segment: the instance's two vertices are its ends,
// and gl_VertexID picks a corner of a quad (a 4-vertex triangle strip).
//
// Each end is first moved toward the eye by uLift times its size in pixels,
// converted to world units at its depth. Moving along the eye ray leaves its
// screen position unchanged, but keeps a line lying on a surface from sinking
// into it.
//
// The segment is then clipped to the near plane and expanded into a quad of
// its thickness in pixels, extended by half the thickness at each end so the
// segments of a polyline overlap at the joints. A segment wholly behind the
// near plane, or too short to have a direction, collapses to a point.
inline constexpr const char* kLineVertex = R"(
layout(location = 0) in vec3 aPositionA;
layout(location = 1) in vec4 aColorA;
layout(location = 2) in float aSizeA;
layout(location = 3) in vec3 aPositionB;
layout(location = 4) in vec4 aColorB;
layout(location = 5) in float aSizeB;

uniform mat4 uView;
uniform mat4 uProjection;
uniform vec2 uViewportSize;
uniform float uLift;

out vec4 vColor;

vec4 Lift(vec3 worldPosition, float size)
{
    vec4 position = uView * vec4(worldPosition, 1.0);
    float w = (uProjection * position).w;
    // The world size of one pixel at this depth (w is 1 in orthographic).
    float pixel = 2.0 * w / (uProjection[1][1] * uViewportSize.y);
    bool perspective = uProjection[3][3] == 0.0;
    vec3 toEye = perspective && dot(position.xyz, position.xyz) > 0.0 ? normalize(-position.xyz) : vec3(0.0, 0.0, 1.0);
    position.xyz += toEye * (uLift * size * max(pixel, 0.0));
    return uProjection * position;
}

void main()
{
    // Corners 0 and 1 are at end a, 2 and 3 at end b; even corners on the
    // left of the segment, odd ones on the right.
    bool atB = gl_VertexID >= 2;
    vColor = atB ? aColorB : aColorA;
    gl_Position = vec4(0.0, 0.0, 0.0, 1.0);

    vec4 a = Lift(aPositionA, aSizeA);
    vec4 b = Lift(aPositionB, aSizeB);
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

    vec4 clip = atB ? b : a;
    vec2 screen = atB ? sb : sa;
    float halfSize = 0.5 * (atB ? aSizeB : aSizeA);
    float side = (gl_VertexID & 1) == 0 ? 1.0 : -1.0;
    float along = atB ? 1.0 : -1.0;
    screen += halfSize * (side * n + along * d);
    gl_Position = vec4(screen / (0.5 * uViewportSize) * clip.w, clip.z, clip.w);
}
)";

inline constexpr const char* kLineFragment = R"(
in vec4 vColor;

out vec4 fragColor;

void main()
{
    fragColor = vColor;
}
)";

// Points, one instance per point: gl_VertexID picks a corner of a square (a
// 4-vertex triangle strip) of twice the point's radius in pixels, with a local
// coordinate in [-1, 1]^2 for the fragment shader to cut a disc from.
//
// The point is first moved toward the eye by uLift times its size in pixels,
// as kLineVertex does for line ends. A point behind the eye or the near plane
// collapses to a point and draws nothing.
inline constexpr const char* kPointVertex = R"(
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec4 aColor;
layout(location = 2) in float aSize;

uniform mat4 uView;
uniform mat4 uProjection;
uniform vec2 uViewportSize;
uniform float uLift;

out vec4 vColor;
out vec2 vLocal;
out float vRadius;

void main()
{
    vec2 corner = vec2((gl_VertexID & 1) == 0 ? -1.0 : 1.0, (gl_VertexID & 2) == 0 ? -1.0 : 1.0);
    vColor = aColor;
    vLocal = corner;
    vRadius = aSize;
    gl_Position = vec4(0.0, 0.0, 0.0, 1.0);

    vec4 position = uView * vec4(aPosition, 1.0);
    float w = (uProjection * position).w;
    // The world size of one pixel at this depth (w is 1 in orthographic).
    float pixel = 2.0 * w / (uProjection[1][1] * uViewportSize.y);
    bool perspective = uProjection[3][3] == 0.0;
    vec3 toEye = perspective && dot(position.xyz, position.xyz) > 0.0 ? normalize(-position.xyz) : vec3(0.0, 0.0, 1.0);
    position.xyz += toEye * (uLift * aSize * max(pixel, 0.0));

    vec4 c = uProjection * position;
    if (c.w <= 0.0 || c.z + c.w < 0.0)
        return;

    vec2 offset = aSize / (0.5 * uViewportSize) * c.w;
    gl_Position = vec4(c.xy + corner * offset, c.z, c.w);
}
)";

// A disc whose edge fades out over about 1.5 pixels.
inline constexpr const char* kPointFragment = R"(
in vec4 vColor;
in vec2 vLocal;
in float vRadius;

out vec4 fragColor;

void main()
{
    float alpha = 1.0 - smoothstep(1.0 - 1.5 / max(vRadius, 1.0), 1.0, length(vLocal));
    if (alpha <= 0.0)
        discard;
    fragColor = vec4(vColor.rgb, vColor.a * alpha);
}
)";

} // namespace Graphics::Shaders
