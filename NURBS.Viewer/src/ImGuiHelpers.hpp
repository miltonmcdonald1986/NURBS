#pragma once

#include <glm/vec2.hpp>
#include <imgui.h>

#include <array>
#include <cstddef>
#include <format>
#include <initializer_list>
#include <iterator>

namespace NURBS::Viewer
{

// A short label held by value, so it can be built inline in a call:
//     view.Label(p, IndexedLabel("a", i).c_str(), color);
struct Label16
{
    std::array<char, 16> text{};

    [[nodiscard]] const char* c_str() const { return text.data(); }
};

// prefix followed by i, e.g. IndexedLabel("C", 3) is "C3".
[[nodiscard]] inline Label16 IndexedLabel(const char* prefix, std::size_t i)
{
    Label16 label;
    std::format_to_n(label.text.data(), label.text.size() - 1, "{}{}", prefix, i);
    return label;
}

// A 2D point as text, e.g. "(1.000, -0.500)".
inline void TextPoint(glm::dvec2 p)
{
    ImGui::Text("(%.3f, %.3f)", p.x, p.y);
}

// The "Fit view" button and navigation hint that end each scene's panel.
// Returns true when the button was clicked.
[[nodiscard]] inline bool FitViewFooter()
{
    ImGui::Spacing();
    const bool clicked = ImGui::Button("Fit view");
    ImGui::TextDisabled("Right/middle drag: pan   Wheel: zoom");
    return clicked;
}

// A slider for a parameter in [0, 1], clamped even when a value is typed in.
// Returns true when the value changed.
inline bool UnitSlider(const char* label, double& value)
{
    static constexpr double kZero = 0.0;
    static constexpr double kOne = 1.0;
    return ImGui::SliderScalar(label, ImGuiDataType_Double, &value, &kZero, &kOne, "%.3f", ImGuiSliderFlags_AlwaysClamp);
}

// The "Samples" slider for the number of points a curve is sampled at.
// Returns true when the count changed.
inline bool SampleCountSlider(int& count)
{
    return ImGui::SliderInt("Samples", &count, 2, 1000, "%d", ImGuiSliderFlags_AlwaysClamp);
}

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

// A combo over items, each shown as nameOf(item). Returns true when the user
// picks an item other than items[selected], which is then stored in selected.
template <typename Range, typename NameOf>
bool NamedCombo(const char* label, const Range& items, std::size_t& selected, NameOf nameOf)
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

// A "Preset" NamedCombo with a "Reset" button beside it, together as wide as a
// normal item. Returns true when presets[selected] should be loaded: another
// preset was picked, or Reset was clicked to undo edits to the current one.
template <typename Range, typename NameOf>
bool PresetCombo(const Range& presets, std::size_t& selected, NameOf nameOf)
{
    const ImGuiStyle& style = ImGui::GetStyle();
    const float resetWidth = ImGui::CalcTextSize("Reset").x + 2.0f * style.FramePadding.x;
    ImGui::SetNextItemWidth(ImGui::CalcItemWidth() - resetWidth - style.ItemInnerSpacing.x);
    bool load = NamedCombo("##preset", presets, selected, nameOf);

    ImGui::SameLine(0.0f, style.ItemInnerSpacing.x);
    if (ImGui::Button("Reset"))
        load = true;
    ImGui::SetItemTooltip("Reload the preset, undoing edits to its points and degree");

    ImGui::SameLine(0.0f, style.ItemInnerSpacing.x);
    ImGui::TextUnformatted("Preset");
    return load;
}

} // namespace NURBS::Viewer
