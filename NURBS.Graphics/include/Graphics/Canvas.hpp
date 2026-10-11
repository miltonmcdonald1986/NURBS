#pragma once

#include <glm/vec4.hpp>
#include <imgui.h>

#include <optional>

namespace Graphics
{

// An interactive region over the current ImGui window's remaining content,
// as Viewport2D and Viewport3D draw into.
struct Canvas
{
    ImDrawList* drawList = nullptr;
    ImVec2 min{};
    ImVec2 size{};
    bool hovered = false;

    [[nodiscard]] ImVec2 Max() const { return {min.x + size.x, min.y + size.y}; }
    // width / height.
    [[nodiscard]] double Aspect() const { return static_cast<double>(size.x) / static_cast<double>(size.y); }
};

// Claims the remaining content region (at least one pixel each way) as an
// invisible button that takes the left, right, and middle mouse buttons, and
// the mouse wheel while hovered so it doesn't also scroll an enclosing window.
// ImGui::IsItemActive then refers to the canvas.
[[nodiscard]] Canvas BeginCanvas(const char* id);

// text beside the screen point anchor, with an optional shadow one pixel
// down and right.
void DrawLabel(ImDrawList* drawList, ImVec2 anchor, const char* text, ImU32 color,
               std::optional<ImU32> shadow = std::nullopt);

// An ImGui color as RGBA floats in [0, 1].
[[nodiscard]] glm::vec4 ToVec4(ImU32 color);

} // namespace Graphics
