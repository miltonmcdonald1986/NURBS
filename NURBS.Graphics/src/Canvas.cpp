#include <Graphics/Canvas.hpp>

#include <algorithm>

namespace Graphics
{

namespace
{

constexpr float kMinCanvasPixels = 1.0f;

} // namespace

Canvas BeginCanvas(const char* id)
{
    Canvas canvas;
    canvas.drawList = ImGui::GetWindowDrawList();
    canvas.min = ImGui::GetCursorScreenPos();
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    canvas.size = ImVec2(std::max(avail.x, kMinCanvasPixels), std::max(avail.y, kMinCanvasPixels));

    ImGui::InvisibleButton(id, canvas.size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    canvas.hovered = ImGui::IsItemHovered();
    if (canvas.hovered)
        ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
    return canvas;
}

void DrawLabel(ImDrawList* drawList, ImVec2 anchor, const char* text, ImU32 color, std::optional<ImU32> shadow)
{
    const ImVec2 at(anchor.x + 6.0f, anchor.y - ImGui::GetFontSize() - 2.0f);
    if (shadow)
        drawList->AddText(ImVec2(at.x + 1.0f, at.y + 1.0f), *shadow, text);
    drawList->AddText(at, color, text);
}

glm::vec4 ToVec4(ImU32 color)
{
    const ImVec4 c = ImGui::ColorConvertU32ToFloat4(color);
    return {c.x, c.y, c.z, c.w};
}

} // namespace Graphics
