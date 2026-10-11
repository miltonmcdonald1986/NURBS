#pragma once

#include "GlHandle.hpp"
#include "OrbitCamera.hpp"
#include "PrimitiveBatch.hpp"
#include "ShaderProgram.hpp"
#include "TriangleMesh.hpp"

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>

#include <optional>
#include <span>

namespace NURBS::Viewer
{

// A 3D canvas over the current ImGui window's remaining content region, seen
// through an OrbitCamera (z up). The scene renders with OpenGL into a
// multisampled offscreen framebuffer (color + depth), which is resolved into a
// texture and shown as an image.
//
// Meshes, lines, and points are drawn with GL and depth tested against each
// other; only text labels are ImDrawList overlays on top.
//
// Usage, each frame:
//     view.Begin("id");
//     ... DrawMesh, Polyline, Line, Point, Label, ... ...
//     view.End();
//
// Input: left drag orbits, right or middle drag pans, the mouse wheel zooms.
//
// GL objects are created on the first Begin, so the GL context must be
// current then, and still current when the viewport is destroyed.
class Viewport3D
{
public:
    void Begin(const char* id);
    void End();

    // Frames the given world-space box on the next Begin (the canvas size is
    // only known then).
    void FitTo(glm::dvec3 min, glm::dvec3 max);

    [[nodiscard]] OrbitCamera& Camera() { return m_camera; }
    [[nodiscard]] const OrbitCamera& Camera() const { return m_camera; }
    [[nodiscard]] bool IsHovered() const { return m_hovered; }

    // The screen position of p, or nullopt if p is behind the camera.
    [[nodiscard]] std::optional<ImVec2> ToScreen(glm::dvec3 p) const;

    // Draws mesh lit by a light at the eye, frontColor on the side its
    // normals face and backColor on the other.
    void DrawMesh(const TriangleMesh& mesh, glm::vec3 frontColor, glm::vec3 backColor);

    // Lines and points, collected during the frame and drawn in End.
    // Positions are in world coordinates and sizes in pixels. Lines and
    // points lying on a surface stay in front of it.
    void Polyline(std::span<const glm::dvec3> points, ImU32 color, float thickness = 2.0f);
    void Line(glm::dvec3 a, glm::dvec3 b, ImU32 color, float thickness = 1.0f);
    void Point(glm::dvec3 p, ImU32 color, float radius = 4.0f);
    // Text beside p, drawn over the GL image (never hidden).
    void Label(glm::dvec3 p, const char* text, ImU32 color) const;
    // A Point with a Label beside it, both in color.
    void LabeledPoint(glm::dvec3 p, const char* text, ImU32 color, float radius = 4.0f);

private:
    struct Box
    {
        glm::dvec3 min;
        glm::dvec3 max;
    };

    void ApplyPendingFit();
    void HandleInput();
    // Creates the framebuffers on first use and sizes them to width x height
    // pixels. Returns false if either is incomplete.
    bool PrepareFramebuffers(int width, int height);
    // Builds the programs on first use. Returns false if any failed.
    bool PreparePrograms();
    void DrawPrimitives();
    void DrawGrid();

    // Sets the uniforms shared by the line and point programs.
    void SetPrimitiveUniforms(const ShaderProgram& program, float lift) const;
    [[nodiscard]] PrimitiveVertex Vertex(glm::dvec3 p, ImU32 color, float size) const;

    [[nodiscard]] glm::dvec4 ToClip(glm::dvec3 p) const;
    [[nodiscard]] ImVec2 ClipToScreen(glm::dvec4 clip) const;

    OrbitCamera m_camera;
    std::optional<Box> m_pendingFit;

    // The scene renders into the multisampled framebuffer, which End resolves
    // into the single-sample one whose texture ImGui shows.
    GlFramebuffer m_msaaFramebuffer;
    GlRenderbuffer m_msaaColor;
    GlRenderbuffer m_msaaDepth;
    GlFramebuffer m_framebuffer;
    GlTexture m_colorTexture;
    int m_samples = 0;
    int m_framebufferWidth = 0;
    int m_framebufferHeight = 0;
    bool m_framebufferFailed = false;

    // Built on the first Begin with a usable framebuffer.
    ShaderProgram m_meshProgram;
    ShaderProgram m_lineProgram;
    ShaderProgram m_pointProgram;
    bool m_programsFailed = false;

    PrimitiveBatch m_lines;
    PrimitiveBatch m_points;

    // Valid between Begin and End.
    ImDrawList* m_drawList = nullptr;
    ImVec2 m_min{};
    ImVec2 m_size{};
    float m_pixelScale = 1.0f;
    bool m_hovered = false;
    bool m_framebufferBound = false;
    glm::dmat4 m_view{1.0};
    glm::dmat4 m_projection{1.0};
    glm::dmat4 m_viewProjection{1.0};
};

} // namespace NURBS::Viewer
