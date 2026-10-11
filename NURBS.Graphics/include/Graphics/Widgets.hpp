#pragma once

#include <glm/gtc/type_ptr.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <imgui.h>

#include <array>
#include <cstddef>
#include <format>
#include <initializer_list>
#include <iterator>
#include <utility>

namespace Graphics
{

// A short label held by value, so it can be built inline in a call:
//     view.Label(p, IndexedLabel("a", i).c_str(), color);
struct Label16
{
    std::array<char, 16> text{};

    [[nodiscard]] const char* c_str() const { return text.data(); }
};

// The text of format with args, cut to fit a Label16.
template <typename... Args>
[[nodiscard]] Label16 FormatLabel(std::format_string<Args...> format, Args&&... args)
{
    Label16 label;
    std::format_to_n(label.text.data(), label.text.size() - 1, format, std::forward<Args>(args)...);
    return label;
}

// prefix followed by i, e.g. IndexedLabel("C", 3) is "C3".
[[nodiscard]] inline Label16 IndexedLabel(const char* prefix, std::size_t i)
{
    return FormatLabel("{}{}", prefix, i);
}

// prefix followed by i and j, e.g. IndexedLabel("a", 1, 2) is "a1,2".
[[nodiscard]] inline Label16 IndexedLabel(const char* prefix, std::size_t i, std::size_t j)
{
    return FormatLabel("{}{},{}", prefix, i, j);
}

// A 2D point as text, e.g. "(1.000, -0.500)".
inline void TextPoint(glm::dvec2 p)
{
    ImGui::Text("(%.3f, %.3f)", p.x, p.y);
}

// A 3D point as text, e.g. "(1.000, -0.500, 2.000)".
inline void TextPoint(glm::dvec3 p)
{
    ImGui::Text("(%.3f, %.3f, %.3f)", p.x, p.y, p.z);
}

// The width of a button showing text.
[[nodiscard]] inline float ButtonWidth(const char* text)
{
    return ImGui::CalcTextSize(text).x + 2.0f * ImGui::GetStyle().FramePadding.x;
}

// The width left for an item placed beside widgets totalling othersWidth
// (inner spacing between them included), so all together are as wide as one
// normal item.
[[nodiscard]] inline float WidthBeside(float othersWidth)
{
    return ImGui::CalcItemWidth() - othersWidth - ImGui::GetStyle().ItemInnerSpacing.x;
}

// A "Fit view" button with a navigation hint under it, e.g.
// Viewport2D::kNavigationHint. Returns true when the button was clicked.
[[nodiscard]] inline bool FitViewFooter(const char* hint)
{
    ImGui::Spacing();
    const bool clicked = ImGui::Button("Fit view");
    // Wrapped, since a long hint can be wider than the panel.
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("%s", hint);
    ImGui::PopTextWrapPos();
    return clicked;
}

// A Play/Pause button that toggles playing, with buttonTooltip, then a slider
// for value in [min, max], clamped, together as wide as one item. buttonId
// must start with "###", so the button keeps its ID when its text changes.
// Returns true when the slider changed value.
bool PlayPauseSlider(const char* label, const char* buttonId, const char* buttonTooltip, double& value,
                     const double& min, const double& max, bool& playing);

// BeginTable with one column per header and the header row already emitted.
// As with BeginTable, call ImGui::EndTable only if this returns true.
[[nodiscard]] inline bool BeginTableWithHeaders(const char* id, std::initializer_list<const char*> headers)
{
    constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable(id, static_cast<int>(headers.size()), kFlags))
        return false;

    for (const char* header : headers)
        ImGui::TableSetupColumn(header);
    ImGui::TableHeadersRow();
    return true;
}

// Starts a table row with label in its first column, leaving the next column
// current for the value.
inline void TableRowLabel(const char* label)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(label);
    ImGui::TableNextColumn();
}

// A table row of label and the point p.
inline void TableRowPoint(const char* label, glm::dvec2 p)
{
    TableRowLabel(label);
    TextPoint(p);
}

// A drag field for the 2D point p. Returns true when p changed.
inline bool DragPoint2(const char* label, glm::dvec2& p)
{
    return ImGui::DragScalarN(label, ImGuiDataType_Double, glm::value_ptr(p), 2, 0.01f, nullptr, nullptr, "%.3f");
}

// The name member of item, the default for NamedCombo's nameOf.
struct NameMember
{
    template <typename T>
    [[nodiscard]] const char* operator()(const T& item) const
    {
        return item.name;
    }
};

// A combo over items, each shown as nameOf(item). Returns true when the user
// picks an item other than items[selected], which is then stored in selected.
template <typename Range, typename NameOf = NameMember>
bool NamedCombo(const char* label, const Range& items, std::size_t& selected, NameOf nameOf = {})
{
    if (!ImGui::BeginCombo(label, nameOf(items[selected])))
        return false;

    bool changed = false;
    for (std::size_t i = 0; i < std::size(items); ++i)
    {
        const bool isSelected = i == selected;
        if (ImGui::Selectable(nameOf(items[i]), isSelected) && !isSelected)
        {
            selected = i;
            changed = true;
        }
        if (isSelected)
            ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
    return changed;
}

} // namespace Graphics
