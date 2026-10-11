/// @file
/// The Canvas region that the viewports draw into, and ImGui drawing helpers.

#pragma once

#include <glm/vec4.hpp>
#include <imgui.h>

#include <optional>

namespace Graphics
{

/// An interactive region over the current ImGui window's remaining content,
/// as Viewport2D and Viewport3D draw into.
struct Canvas
{
    /// The draw list of the window the canvas is in.
    ImDrawList* drawList = nullptr;
    /// The top-left corner, in screen coordinates.
    ImVec2 min{};
    /// The width and height, in pixels.
    ImVec2 size{};
    /// Whether the mouse is over the canvas.
    bool hovered = false;

    /// The bottom-right corner.
    /// @return The bottom-right corner, in screen coordinates.
    [[nodiscard]] ImVec2 Max() const { return {min.x + size.x, min.y + size.y}; }
    /// The aspect ratio.
    /// @return width / height.
    [[nodiscard]] double Aspect() const { return static_cast<double>(size.x) / static_cast<double>(size.y); }
};

/// Claims the remaining content region (at least one pixel each way) as an
/// invisible button that takes the left, right, and middle mouse buttons, and
/// the mouse wheel while hovered so it doesn't also scroll an enclosing window.
/// ImGui::IsItemActive then refers to the canvas.
/// @param id The ImGui ID of the invisible button.
/// @return The claimed region.
[[nodiscard]] Canvas BeginCanvas(const char* id);

/// Draws @p text beside the screen point @p anchor, with an optional shadow
/// one pixel down and right.
/// @param drawList The draw list to draw into.
/// @param anchor The screen point the text is drawn beside.
/// @param text The text.
/// @param color The text color.
/// @param shadow The shadow color, or no shadow if empty.
void DrawLabel(ImDrawList* drawList, ImVec2 anchor, const char* text, ImU32 color,
               std::optional<ImU32> shadow = std::nullopt);

/// Converts an ImGui color to RGBA floats.
/// @param color The ImGui color.
/// @return The color as RGBA floats in [0, 1].
[[nodiscard]] glm::vec4 ToVec4(ImU32 color);

} // namespace Graphics
