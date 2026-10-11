/// @file
/// Labels and ImGui widgets shared by scenes.

#pragma once

#include "OrbitCamera.hpp"

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

/// A short label held by value, so it can be built inline in a call:
/// @code
///     view.Label(p, IndexedLabel("a", i).c_str(), color);
/// @endcode
struct Label16
{
    /// The null-terminated text.
    std::array<char, 16> text{};

    /// The text as a C string.
    /// @return The null-terminated text, valid while the label lives.
    [[nodiscard]] const char* c_str() const { return text.data(); }
};

/// Formats a label.
/// @tparam Args The argument types.
/// @param format The `std::format` format string.
/// @param args The arguments to format.
/// @return The text of @p format with @p args, cut to fit a Label16.
template <typename... Args>
[[nodiscard]] Label16 FormatLabel(std::format_string<Args...> format, Args&&... args)
{
    Label16 label;
    std::format_to_n(label.text.data(), label.text.size() - 1, format, std::forward<Args>(args)...);
    return label;
}

/// A label with one index.
/// @param prefix The text before the index.
/// @param i The index.
/// @return @p prefix followed by @p i, e.g. `IndexedLabel("C", 3)` is "C3".
[[nodiscard]] inline Label16 IndexedLabel(const char* prefix, std::size_t i)
{
    return FormatLabel("{}{}", prefix, i);
}

/// A label with two indices.
/// @param prefix The text before the indices.
/// @param i The first index.
/// @param j The second index.
/// @return @p prefix followed by @p i and @p j, e.g. `IndexedLabel("a", 1, 2)` is "a1,2".
[[nodiscard]] inline Label16 IndexedLabel(const char* prefix, std::size_t i, std::size_t j)
{
    return FormatLabel("{}{},{}", prefix, i, j);
}

/// Shows a 2D point as text, e.g. "(1.000, -0.500)".
/// @param p The point.
inline void TextPoint(glm::dvec2 p)
{
    ImGui::Text("(%.3f, %.3f)", p.x, p.y);
}

/// Shows a 3D point as text, e.g. "(1.000, -0.500, 2.000)".
/// @param p The point.
inline void TextPoint(glm::dvec3 p)
{
    ImGui::Text("(%.3f, %.3f, %.3f)", p.x, p.y, p.z);
}

/// The width of a button showing @p text.
/// @param text The button's text.
/// @return The width, in pixels.
[[nodiscard]] inline float ButtonWidth(const char* text)
{
    return ImGui::CalcTextSize(text).x + 2.0f * ImGui::GetStyle().FramePadding.x;
}

/// The width left for an item placed beside widgets totalling @p othersWidth
/// (inner spacing between them included), so all together are as wide as one
/// normal item.
/// @param othersWidth The total width of the other widgets, in pixels.
/// @return The width, in pixels.
[[nodiscard]] inline float WidthBeside(float othersWidth)
{
    return ImGui::CalcItemWidth() - othersWidth - ImGui::GetStyle().ItemInnerSpacing.x;
}

/// A "Fit view" button with a navigation hint under it.
/// @param hint The hint, e.g. Viewport2D::kNavigationHint.
/// @return True when the button was clicked.
[[nodiscard]] inline bool FitViewButton(const char* hint)
{
    const bool clicked = ImGui::Button("Fit view");
    // Wrapped, since a long hint can be wider than the window.
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("%s", hint);
    ImGui::PopTextWrapPos();
    return clicked;
}

/// A "Translucent" checkbox and, while it is checked, an "Opacity" slider.
/// @param translucent Whether the mesh is drawn translucent, updated in place.
/// @param opacity The opacity used when translucent, updated in place.
inline void TranslucencyControls(bool& translucent, float& opacity)
{
    ImGui::Checkbox("Translucent", &translucent);
    ImGui::SetItemTooltip("Show what lies behind the surface through it");
    if (translucent)
        ImGui::SliderFloat("Opacity", &opacity, 0.05f, 0.95f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
}

/// An "Orthographic" checkbox that switches @p camera's projection.
/// @param camera The camera.
inline void ProjectionCheckbox(OrbitCamera& camera)
{
    bool orthographic = camera.IsOrthographic();
    if (ImGui::Checkbox("Orthographic", &orthographic))
        camera.SetOrthographic(orthographic);
}

/// A Play/Pause button that toggles @p playing, with @p buttonTooltip, then a
/// slider for @p value in [@p min, @p max], clamped, together as wide as one item.
/// @param label The slider's label.
/// @param buttonId The button's ID. It must start with "###", so the button
///                 keeps its ID when its text changes.
/// @param buttonTooltip The button's tooltip.
/// @param value The slider's value, updated in place.
/// @param min The slider's minimum.
/// @param max The slider's maximum.
/// @param playing Whether playing, toggled in place by the button.
/// @return True when the slider changed @p value.
bool PlayPauseSlider(const char* label, const char* buttonId, const char* buttonTooltip, double& value,
                     const double& min, const double& max, bool& playing);

/// ImGui::BeginTable with one column per header and the header row already
/// emitted. As with ImGui::BeginTable, call ImGui::EndTable only if this
/// returns true.
/// @param id The table's ImGui ID.
/// @param headers The column headers.
/// @return True if the table is visible.
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

/// Starts a table row with @p label in its first column, leaving the next
/// column current for the value.
/// @param label The label.
inline void TableRowLabel(const char* label)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(label);
    ImGui::TableNextColumn();
}

/// Adds a table row of @p label and the point @p p.
/// @param label The label.
/// @param p The point.
inline void TableRowPoint(const char* label, glm::dvec2 p)
{
    TableRowLabel(label);
    TextPoint(p);
}

/// A drag field for the 2D point @p p.
/// @param label The field's label.
/// @param p The point, updated in place.
/// @return True when @p p changed.
inline bool DragPoint2(const char* label, glm::dvec2& p)
{
    return ImGui::DragScalarN(label, ImGuiDataType_Double, glm::value_ptr(p), 2, 0.01f, nullptr, nullptr, "%.3f");
}

/// Gives an item's `name` member, the default for NamedCombo's nameOf.
struct NameMember
{
    /// The name of @p item.
    /// @tparam T A type with a `const char* name` member.
    /// @param item The item.
    /// @return `item.name`.
    template <typename T>
    [[nodiscard]] const char* operator()(const T& item) const
    {
        return item.name;
    }
};

/// A combo over @p items, each shown as `nameOf(item)`.
/// @tparam Range A random-access range of items.
/// @tparam NameOf A callable taking an item and returning its `const char*` name.
///                By default, the item's `name` member.
/// @param label The combo's label.
/// @param items The items.
/// @param selected The index of the selected item, updated in place.
/// @param nameOf Gives each item's name.
/// @return True when the user picks an item other than `items[selected]`,
///         which is then stored in @p selected.
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
