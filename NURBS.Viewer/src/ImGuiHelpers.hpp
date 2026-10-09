#pragma once

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

// The "Fit view" button and navigation hint that end each scene's panel.
// Returns true when the button was clicked.
[[nodiscard]] inline bool FitViewFooter()
{
    ImGui::Spacing();
    const bool clicked = ImGui::Button("Fit view");
    ImGui::TextDisabled("Right/middle drag: pan   Wheel: zoom");
    return clicked;
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

} // namespace NURBS::Viewer
