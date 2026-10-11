/// @file
/// Viewport2D, a pannable, zoomable 2D canvas.

#pragma once

#include "Bounds.hpp"
#include "Canvas.hpp"

#include <glm/vec2.hpp>
#include <imgui.h>

#include <optional>
#include <span>

namespace Graphics
{

/// A pannable, zoomable 2D canvas over the current ImGui window's remaining
/// content region, with world coordinates in glm::dvec2 (y up).
///
/// Usage, each frame:
/// @code
///     view.Begin("id");
///     view.Polyline(points, color);
///     view.End();
/// @endcode
///
/// Input: right or middle drag pans, the mouse wheel zooms about the cursor.
/// The left button is left for the caller (see DragPoints, or IsHovered /
/// MouseWorld for custom handling).
class Viewport2D
{
public:
    /// A hint for the controls above, e.g. for FitViewFooter.
    static constexpr const char* kNavigationHint = "Right/middle drag: pan   Wheel: zoom";

    /// Claims the canvas, handles navigation input, and draws the background
    /// and grid. Call once per frame, before any drawing.
    /// @param id The canvas's ImGui ID.
    void Begin(const char* id);
    /// Ends the frame's drawing.
    void End();

    /// Frames the given world-space box on the next Begin (the canvas size is
    /// only known then).
    /// @param min The box's minimum corner.
    /// @param max The box's maximum corner.
    void FitTo(glm::dvec2 min, glm::dvec2 max);

    /// Converts a world point to screen coordinates.
    /// @param p The world point.
    /// @return The screen point.
    [[nodiscard]] ImVec2 ToScreen(glm::dvec2 p) const;
    /// Converts a screen point to world coordinates.
    /// @param p The screen point.
    /// @return The world point.
    [[nodiscard]] glm::dvec2 ToWorld(ImVec2 p) const;
    /// The mouse position in world coordinates.
    /// @return The world point under the mouse.
    [[nodiscard]] glm::dvec2 MouseWorld() const;
    /// Whether the mouse is over the canvas.
    /// @return True if the canvas is hovered.
    [[nodiscard]] bool IsHovered() const { return m_canvas.hovered; }
    /// The zoom level.
    /// @return The number of pixels per world unit.
    [[nodiscard]] double PixelsPerUnit() const { return m_pixelsPerUnit; }

    /// Left-drag editing of points, between Begin and End. A left click within
    /// @p radius pixels of a point grabs the nearest one into @p active; while
    /// the button is held it follows the mouse (even outside the canvas), and
    /// releasing sets @p active to -1.
    /// @param points The points, moved in place.
    /// @param active The index of the grabbed point, or -1 if none.
    /// @param radius The grab radius, in pixels.
    /// @return True if a point moved this frame.
    bool DragPoints(std::span<glm::dvec2> points, int& active, float radius = 7.0f) const;

    /// Draws a polyline through @p points.
    /// @param points The vertices, in world coordinates.
    /// @param color The color.
    /// @param thickness The thickness, in pixels.
    void Polyline(std::span<const glm::dvec2> points, ImU32 color, float thickness = 2.0f) const;
    /// Draws a line segment.
    /// @param a The start, in world coordinates.
    /// @param b The end, in world coordinates.
    /// @param color The color.
    /// @param thickness The thickness, in pixels.
    void Line(glm::dvec2 a, glm::dvec2 b, ImU32 color, float thickness = 1.0f) const;
    /// Draws an arrow.
    /// @param from The tail, in world coordinates.
    /// @param to The head, in world coordinates.
    /// @param color The color.
    /// @param thickness The thickness, in pixels.
    void Arrow(glm::dvec2 from, glm::dvec2 to, ImU32 color, float thickness = 1.5f) const;
    /// Draws a filled circle.
    /// @param p The center, in world coordinates.
    /// @param color The color.
    /// @param radius The radius, in pixels.
    void Point(glm::dvec2 p, ImU32 color, float radius = 4.0f) const;
    /// Draws a filled axis-aligned rectangle with opposite corners @p a and @p b.
    /// @param a One corner, in world coordinates.
    /// @param b The opposite corner, in world coordinates.
    /// @param color The color.
    void Rect(glm::dvec2 a, glm::dvec2 b, ImU32 color) const;
    /// Draws text beside a point.
    /// @param p The point, in world coordinates.
    /// @param text The text.
    /// @param color The color.
    void Label(glm::dvec2 p, const char* text, ImU32 color) const;
    /// Draws a Point with a Label beside it, both in @p color.
    /// @param p The point, in world coordinates.
    /// @param text The text.
    /// @param color The color.
    /// @param radius The point's radius, in pixels.
    void LabeledPoint(glm::dvec2 p, const char* text, ImU32 color, float radius = 4.0f) const;

private:
    void ApplyPendingFit();
    void HandleInput();
    void DrawGrid() const;
    // The index of the point nearest the mouse within radius pixels, or -1.
    [[nodiscard]] int NearestToMouse(std::span<const glm::dvec2> points, float radius) const;

    // The screen position of the canvas center, where m_center is drawn.
    [[nodiscard]] glm::dvec2 ScreenCenter() const;

    glm::dvec2 m_center{0.0, 0.0};
    double m_pixelsPerUnit = 100.0;
    std::optional<Bounds2D> m_pendingFit;

    // Valid between Begin and End.
    Canvas m_canvas;
};

} // namespace Graphics
