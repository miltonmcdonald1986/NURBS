/// @file
/// Viewport3D, an orbitable 3D canvas rendered with OpenGL.

#pragma once

#include "Bounds.hpp"
#include "Canvas.hpp"
#include "MultisampleTarget.hpp"
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
#include <vector>

namespace Graphics
{

/// A 3D canvas over the current ImGui window's remaining content region, seen
/// through an OrbitCamera (z up). The scene renders with OpenGL into a
/// MultisampleTarget, whose resolved texture is shown as an image.
///
/// Meshes, lines, and points are drawn with GL and depth tested against each
/// other; only text labels are ImDrawList overlays on top.
///
/// Usage, each frame:
/// @code
///     view.Begin("id");
///     ... DrawMesh, Polyline, Line, Point, Label, ... ...
///     view.End();
/// @endcode
///
/// Input: left drag orbits, right or middle drag pans, the mouse wheel zooms.
///
/// GL objects are created on the first Begin, so the GL context must be
/// current then, and still current when the viewport is destroyed.
class Viewport3D
{
public:
    /// A hint for the controls above, e.g. for FitViewFooter.
    static constexpr const char* kNavigationHint = "Left drag: orbit   Right/middle drag: pan   Wheel: zoom";

    /// Claims the canvas, handles navigation input, and starts rendering.
    /// Call once per frame, before any drawing.
    /// @param id The canvas's ImGui ID.
    void Begin(const char* id);
    /// Draws the collected lines, points, and translucent meshes, and shows
    /// the rendered image with its overlays.
    void End();

    /// Frames the given world-space box on the next Begin (the canvas size is
    /// only known then).
    /// @param min The box's minimum corner.
    /// @param max The box's maximum corner.
    void FitTo(glm::dvec3 min, glm::dvec3 max);

    /// The camera.
    /// @return The camera, which the caller may change.
    [[nodiscard]] OrbitCamera& Camera() { return m_camera; }
    /// The camera.
    /// @return The camera.
    [[nodiscard]] const OrbitCamera& Camera() const { return m_camera; }
    /// Whether the mouse is over the canvas.
    /// @return True if the canvas is hovered.
    [[nodiscard]] bool IsHovered() const { return m_canvas.hovered; }

    /// Projects a world point to the screen.
    /// @param p The world point.
    /// @return The screen position of @p p, or `std::nullopt` if @p p is behind the camera.
    [[nodiscard]] std::optional<ImVec2> ToScreen(glm::dvec3 p) const;

    /// Draws @p mesh lit by a light at the eye, @p frontColor on the side its
    /// normals face and @p backColor on the other. An opacity below 1 blends
    /// the mesh over everything else, drawn in End, so @p mesh must stay alive
    /// until then; overlapping translucent layers are blended unsorted.
    /// @param mesh The mesh.
    /// @param frontColor The RGB color of the side the normals face.
    /// @param backColor The RGB color of the other side.
    /// @param opacity The opacity, in [0, 1].
    void DrawMesh(const TriangleMesh& mesh, glm::vec3 frontColor, glm::vec3 backColor, float opacity = 1.0f);

    /// Adds a polyline through @p points.
    ///
    /// Lines and points are collected during the frame and drawn in End.
    /// Lines and points lying on a surface stay in front of it.
    /// @param points The vertices, in world coordinates.
    /// @param color The color.
    /// @param thickness The thickness, in pixels.
    void Polyline(std::span<const glm::dvec3> points, ImU32 color, float thickness = 2.0f);
    /// Adds a line segment, drawn in End.
    /// @param a The start, in world coordinates.
    /// @param b The end, in world coordinates.
    /// @param color The color.
    /// @param thickness The thickness, in pixels.
    void Line(glm::dvec3 a, glm::dvec3 b, ImU32 color, float thickness = 1.0f);
    /// Adds a point, drawn in End.
    /// @param p The center, in world coordinates.
    /// @param color The color.
    /// @param radius The radius, in pixels.
    void Point(glm::dvec3 p, ImU32 color, float radius = 4.0f);
    /// Draws text beside @p p, over the GL image (never hidden).
    /// @param p The point, in world coordinates.
    /// @param text The text.
    /// @param color The color.
    void Label(glm::dvec3 p, const char* text, ImU32 color) const;
    /// Draws a Point with a Label beside it, both in @p color.
    /// @param p The point, in world coordinates.
    /// @param text The text.
    /// @param color The color.
    /// @param radius The point's radius, in pixels.
    void LabeledPoint(glm::dvec3 p, const char* text, ImU32 color, float radius = 4.0f);

private:
    void ApplyPendingFit();
    void HandleInput();
    void UpdateMatrices();
    // Sizes and binds the render target and clears it. Returns false if the
    // target is unusable.
    bool BeginRenderTarget();
    // Builds the programs on first use. Returns false if any failed.
    bool PreparePrograms();
    void DrawPrimitives();
    void DrawTranslucentMeshes();
    void DrawGrid();

    // Draws mesh with the mesh program, which must be prepared.
    void RenderMesh(const TriangleMesh& mesh, glm::vec3 frontColor, glm::vec3 backColor, float opacity) const;

    // Sets the uniforms shared by the line and point programs.
    void SetPrimitiveUniforms(const ShaderProgram& program, float lift) const;
    [[nodiscard]] PrimitiveVertex Vertex(glm::dvec3 p, ImU32 color, float size) const;

    OrbitCamera m_camera;
    std::optional<Bounds3D> m_pendingFit;

    MultisampleTarget m_target;

    // Built on the first Begin with a usable render target.
    ShaderProgram m_meshProgram;
    ShaderProgram m_lineProgram;
    ShaderProgram m_pointProgram;
    bool m_programsFailed = false;

    PrimitiveBatch m_lines;
    PrimitiveBatch m_points;

    struct TranslucentMesh
    {
        const TriangleMesh* mesh;
        glm::vec3 frontColor;
        glm::vec3 backColor;
        float opacity;
    };
    std::vector<TranslucentMesh> m_translucentMeshes;

    // Valid between Begin and End.
    Canvas m_canvas;
    float m_pixelScale = 1.0f;
    bool m_targetBound = false;
    glm::dmat4 m_view{1.0};
    glm::dmat4 m_projection{1.0};
    glm::dmat4 m_viewProjection{1.0};
};

} // namespace Graphics
