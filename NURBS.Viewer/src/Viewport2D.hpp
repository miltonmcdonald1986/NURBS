#pragma once

#include <glm/vec2.hpp>
#include <imgui.h>

#include <optional>
#include <span>

namespace NURBS::Viewer
{

// A pannable, zoomable 2D canvas over the current ImGui window's remaining
// content region, with world coordinates in glm::dvec2 (y up).
//
// Usage, each frame:
//     view.Begin("id");
//     view.Polyline(points, color);
//     view.End();
//
// Input: right or middle drag pans, the mouse wheel zooms about the cursor.
// The left button is left for the caller (see DragPoints, or IsHovered /
// MouseWorld for custom handling).
class Viewport2D
{
public:
    void Begin(const char* id);
    void End();

    // Frames the given world-space box on the next Begin (the canvas size is
    // only known then).
    void FitTo(glm::dvec2 min, glm::dvec2 max);

    [[nodiscard]] ImVec2 ToScreen(glm::dvec2 p) const;
    [[nodiscard]] glm::dvec2 ToWorld(ImVec2 p) const;
    [[nodiscard]] glm::dvec2 MouseWorld() const;
    [[nodiscard]] bool IsHovered() const { return m_hovered; }
    [[nodiscard]] double PixelsPerUnit() const { return m_pixelsPerUnit; }

    // Left-drag editing of points, between Begin and End. A left click within
    // radius pixels of a point grabs the nearest one into active; while the
    // button is held it follows the mouse (even outside the canvas), and
    // releasing sets active to -1. Returns true if a point moved this frame.
    bool DragPoints(std::span<glm::dvec2> points, int& active, float radius = 7.0f) const;

    // Drawing helpers; all positions are in world coordinates and sizes in pixels.
    void Polyline(std::span<const glm::dvec2> points, ImU32 color, float thickness = 2.0f) const;
    void Line(glm::dvec2 a, glm::dvec2 b, ImU32 color, float thickness = 1.0f) const;
    void Arrow(glm::dvec2 from, glm::dvec2 to, ImU32 color, float thickness = 1.5f) const;
    void Point(glm::dvec2 p, ImU32 color, float radius = 4.0f) const;
    void Label(glm::dvec2 p, const char* text, ImU32 color) const;

private:
    struct Box
    {
        glm::dvec2 min;
        glm::dvec2 max;
    };

    void ApplyPendingFit();
    void HandleInput();
    void DrawGrid() const;

    glm::dvec2 m_center{0.0, 0.0};
    double m_pixelsPerUnit = 100.0;
    std::optional<Box> m_pendingFit;

    // Valid between Begin and End.
    ImDrawList* m_drawList = nullptr;
    ImVec2 m_min{};
    ImVec2 m_size{};
    bool m_hovered = false;
};

} // namespace NURBS::Viewer
