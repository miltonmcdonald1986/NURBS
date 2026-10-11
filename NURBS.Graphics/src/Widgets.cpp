#include <Graphics/Widgets.hpp>

#include <array>
#include <cstddef>
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

void StandardViewButtons(OrbitCamera& camera)
{
    struct Button
    {
        const char* name;
        StandardView view;
    };
    // Opposite views stacked, so each column looks along one axis.
    static constexpr std::array<std::array<Button, 3>, 2> kRows{{
        {{{"Top", StandardView::Top}, {"Front", StandardView::Front}, {"Right", StandardView::Right}}},
        {{{"Bottom", StandardView::Bottom}, {"Back", StandardView::Back}, {"Left", StandardView::Left}}},
    }};

    ImGui::PushID("StandardViews");
    // Equal widths, sized for the widest name, with Isometric spanning the row.
    const float width = ButtonWidth("Bottom");
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    for (const auto& row : kRows)
    {
        for (std::size_t i = 0; i < row.size(); ++i)
        {
            if (i > 0)
                ImGui::SameLine();
            if (ImGui::Button(row[i].name, ImVec2(width, 0.0f)))
                camera.SetView(row[i].view);
        }
    }
    if (ImGui::Button("Isometric", ImVec2(3.0f * width + 2.0f * spacing, 0.0f)))
        camera.SetView(StandardView::Isometric);
    ImGui::PopID();
}

} // namespace Graphics
