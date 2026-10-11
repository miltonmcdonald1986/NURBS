#pragma once

#include <Graphics/Widgets.hpp>

#include <imgui.h>

#include <cstddef>

// Controls shared by the viewer's scenes, built on Graphics/Widgets.hpp.

namespace NURBS::Viewer
{

// The "Samples" slider for the number of points a curve is sampled at.
// Returns true when the count changed.
inline bool SampleCountSlider(int& count)
{
    return ImGui::SliderInt("Samples", &count, 2, 1000, "%d", ImGuiSliderFlags_AlwaysClamp);
}

constexpr float kMinSweepSeconds = 1.0f;
constexpr float kMaxSweepSeconds = 20.0f;

// The "Sweep time (s)" slider for the seconds an animation takes to cross its
// domain once. Returns true when the time changed.
inline bool SweepTimeSlider(float& seconds)
{
    const bool changed = ImGui::SliderFloat("Sweep time (s)", &seconds, kMinSweepSeconds, kMaxSweepSeconds, "%.1f",
                                            ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetItemTooltip("Seconds for an animation to cross the domain once");
    return changed;
}

// The share of its domain an animation taking seconds per sweep crosses this frame.
[[nodiscard]] inline double SweepStep(float seconds)
{
    return static_cast<double>(ImGui::GetIO().DeltaTime / seconds);
}

// "<name> min" and "<name> max" inputs for a parameter range, with max kept
// above min after either is edited. Returns true when either changed.
inline bool InputRange(const char* name, double& min, double& max)
{
    const Graphics::Label16 minLabel = Graphics::FormatLabel("{} min", name);
    const Graphics::Label16 maxLabel = Graphics::FormatLabel("{} max", name);
    bool changed = ImGui::InputDouble(minLabel.c_str(), &min, 0.1, 1.0, "%.3f");
    changed |= ImGui::InputDouble(maxLabel.c_str(), &max, 0.1, 1.0, "%.3f");
    if (max <= min)
        max = min + 0.001;
    return changed;
}

// A "Preset" NamedCombo with a "Reset" button beside it, together as wide as a
// normal item. Returns true when presets[selected] should be loaded: another
// preset was picked, or Reset was clicked to undo edits to the current one.
template <typename Range, typename NameOf>
bool PresetCombo(const Range& presets, std::size_t& selected, NameOf nameOf)
{
    const float innerSpacing = ImGui::GetStyle().ItemInnerSpacing.x;
    ImGui::SetNextItemWidth(Graphics::WidthBeside(Graphics::ButtonWidth("Reset")));
    bool load = Graphics::NamedCombo("##preset", presets, selected, nameOf);

    ImGui::SameLine(0.0f, innerSpacing);
    if (ImGui::Button("Reset"))
        load = true;
    ImGui::SetItemTooltip("Reload the preset, undoing edits to its points and degree");

    ImGui::SameLine(0.0f, innerSpacing);
    ImGui::TextUnformatted("Preset");
    return load;
}

} // namespace NURBS::Viewer
