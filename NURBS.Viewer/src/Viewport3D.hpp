#pragma once

#include "GlHandle.hpp"
#include "OrbitCamera.hpp"
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
// through an OrbitCamera (z up). The scene renders with OpenGL into an
// offscreen framebuffer (color + depth) that is shown as an image, and can
// draw overlays (lines, points, labels) on top with ImDrawList.
//
// Usage, each frame:
//     view.Begin("id");
//     ... GL draw calls (the framebuffer is bound) and overlay helpers ...
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

    // Draws mesh lit by a light at the eye, depth tested, frontColor on the
    // side its normals face and backColor on the other. Only between Begin
    // and End, and only if the framebuffer is usable.
    void DrawMesh(const TriangleMesh& mesh, glm::vec3 frontColor, glm::vec3 backColor);

    // Overlay helpers, drawn over the GL image; positions are in world
    // coordinates and sizes in pixels. Lines are clipped where they pass
    // behind the camera.
    void Polyline(std::span<const glm::dvec3> points, ImU32 color, float thickness = 2.0f) const;
    void Line(glm::dvec3 a, glm::dvec3 b, ImU32 color, float thickness = 1.0f) const;
    void Point(glm::dvec3 p, ImU32 color, float radius = 4.0f) const;
    void Label(glm::dvec3 p, const char* text, ImU32 color) const;
    // A Point with a Label beside it, both in color.
    void LabeledPoint(glm::dvec3 p, const char* text, ImU32 color, float radius = 4.0f) const;

private:
    struct Box
    {
        glm::dvec3 min;
        glm::dvec3 max;
    };

    void ApplyPendingFit();
    void HandleInput();
    // Creates the framebuffer on first use and sizes it to width x height
    // pixels. Returns false if it is incomplete.
    bool PrepareFramebuffer(int width, int height);
    void DrawGrid() const;

    [[nodiscard]] glm::dvec4 ToClip(glm::dvec3 p) const;
    [[nodiscard]] ImVec2 ClipToScreen(glm::dvec4 clip) const;

    OrbitCamera m_camera;
    std::optional<Box> m_pendingFit;

    GlFramebuffer m_framebuffer;
    GlTexture m_colorTexture;
    GlRenderbuffer m_depthBuffer;
    int m_framebufferWidth = 0;
    int m_framebufferHeight = 0;
    bool m_framebufferFailed = false;

    // Built on the first DrawMesh.
    ShaderProgram m_meshProgram;
    bool m_meshProgramFailed = false;

    // Valid between Begin and End.
    ImDrawList* m_drawList = nullptr;
    ImVec2 m_min{};
    ImVec2 m_size{};
    bool m_hovered = false;
    bool m_framebufferBound = false;
    glm::dmat4 m_view{1.0};
    glm::dmat4 m_projection{1.0};
    glm::dmat4 m_viewProjection{1.0};
};

} // namespace NURBS::Viewer
