#include <Graphics/Widgets.hpp>

#include <string>

namespace Graphics
{

bool PlayPauseSlider(const char* label, const char* buttonId, const char* buttonTooltip, double& value,
                     const double& min, const double& max, bool& playing)
{
    // Sized for the wider text, so the slider doesn't move when it toggles.
    const float buttonWidth = ButtonWidth("Pause");
    const float sliderWidth = WidthBeside(buttonWidth);

    const std::string buttonLabel = std::string(playing ? "Pause" : "Play") + buttonId;
    if (ImGui::Button(buttonLabel.c_str(), ImVec2(buttonWidth, 0.0f)))
        playing = !playing;
    ImGui::SetItemTooltip("%s", buttonTooltip);

    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::SetNextItemWidth(sliderWidth);
    return ImGui::SliderScalar(label, ImGuiDataType_Double, &value, &min, &max, "%.3f", ImGuiSliderFlags_AlwaysClamp);
}

} // namespace Graphics
