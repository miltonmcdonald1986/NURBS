#include "HornerScene.hpp"

#include "CurveSampling.hpp"
#include "ImGuiHelpers.hpp"

#include <NURBS/horner1.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <span>

namespace NURBS::Viewer
{

namespace
{

constexpr int kMaxDegree = 6;

constexpr ImU32 kCurveColor = IM_COL32(90, 170, 255, 255);
constexpr ImU32 kRayColor = IM_COL32(120, 120, 140, 140);
constexpr ImU32 kScaleColor = IM_COL32(240, 180, 60, 255);
constexpr ImU32 kAddColor = IM_COL32(110, 220, 120, 255);
constexpr ImU32 kChainColor = IM_COL32(230, 230, 230, 255);
constexpr ImU32 kResultColor = IM_COL32(255, 90, 90, 255);
constexpr ImU32 kPowerColor = IM_COL32(200, 120, 255, 255);

struct Preset
{
    const char* name;
    std::vector<glm::dvec2> coefficients;
};

const std::array<Preset, 3>& Presets()
{
    static const std::array<Preset, 3> presets{{
        {"Line", {{-1.0, -0.5}, {2.0, 1.0}}},
        {"Parabola (u, u^2)", {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}},
        // The Bezier arch with control points (0,0), (0,1), (1,1), (1,0), in power basis form.
        {"Cubic arch", {{0.0, 0.0}, {0.0, 3.0}, {3.0, -3.0}, {-2.0, 0.0}}},
    }};
    return presets;
}

} // namespace

HornerScene::HornerScene()
{
    LoadPreset(2);
}

void HornerScene::LoadPreset(std::size_t preset)
{
    m_preset = preset;
    m_coefficients = Presets()[preset].coefficients;
    m_stepsShown = static_cast<int>(m_coefficients.size()) - 1;
    FitView();
    FitCoefficientView();
}

void HornerScene::FitView()
{
    const std::vector<glm::dvec2> curve = SampledCurve();
    Bounds2D bounds = *BoundsOf(curve);
    if (m_showChain)
    {
        // The chain's scaling rays start at the origin.
        bounds.Extend({0.0, 0.0});
        bounds.Extend(HornerChain());
    }
    if (m_showPowerSum)
    {
        bounds.Extend({0.0, 0.0});
        bounds.Extend(PowerSum());
    }
    m_view.FitTo(bounds.min, bounds.max);
}

void HornerScene::FitCoefficientView()
{
    // The vectors all start at the origin. Keep at least a unit box around it
    // so short or zero vectors still leave room to drag.
    Bounds2D bounds = *BoundsOf(m_coefficients);
    bounds.Extend({-0.5, -0.5});
    bounds.Extend({0.5, 0.5});
    m_coefficientView.FitTo(bounds.min, bounds.max);
}

std::vector<glm::dvec2> HornerScene::HornerChain() const
{
    // C_k = sum a[i] * u0^(i-k), i = k..n, which is horner1 applied to the
    // tail a[k..n]; so the library computes every step of the chain.
    const std::span<const glm::dvec2> a(m_coefficients);
    std::vector<glm::dvec2> chain(a.size());
    for (std::size_t k = 0; k < a.size(); ++k)
        chain[k] = NURBS::horner1(a.subspan(k), m_u0);
    return chain;
}

std::vector<glm::dvec2> HornerScene::PowerSum() const
{
    std::vector<glm::dvec2> sums(m_coefficients.size());
    glm::dvec2 sum{0.0, 0.0};
    double power = 1.0;
    for (std::size_t i = 0; i < m_coefficients.size(); ++i)
    {
        sum += power * m_coefficients[i];
        sums[i] = sum;
        power *= m_u0;
    }
    return sums;
}

std::vector<glm::dvec2> HornerScene::SampledCurve() const
{
    return SampleCurve([this](double u) { return NURBS::horner1(m_coefficients, u); }, m_uMin, m_uMax,
                       static_cast<std::size_t>(m_sampleCount));
}

void HornerScene::DrawUI()
{
    if (PresetCombo(Presets(), m_preset, [](const Preset& preset) { return preset.name; }))
        LoadPreset(m_preset);

    int degree = static_cast<int>(m_coefficients.size()) - 1;
    if (ImGui::SliderInt("Degree n", &degree, 0, kMaxDegree, "%d", ImGuiSliderFlags_AlwaysClamp))
    {
        m_coefficients.resize(static_cast<std::size_t>(degree) + 1, glm::dvec2{0.0, 0.0});
        m_stepsShown = std::min(m_stepsShown, degree);
        m_draggedCoefficient = -1;
        FitCoefficientView();
    }

    ImGui::SeparatorText("Coefficients a[i]");
    for (std::size_t i = 0; i < m_coefficients.size(); ++i)
    {
        ImGui::DragScalarN(IndexedLabel("a", i).c_str(), ImGuiDataType_Double, glm::value_ptr(m_coefficients[i]), 2, 0.01f, nullptr, nullptr,
                           "%.3f");
    }

    // A square canvas the width of the panel.
    const float side = ImGui::GetContentRegionAvail().x;
    if (ImGui::BeginChild("##coefficients", ImVec2(side, side), ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
    {
        DrawCoefficientView();
    }
    ImGui::EndChild();
    if (ImGui::Button("Fit vectors"))
        FitCoefficientView();
    ImGui::SameLine();
    ImGui::TextDisabled("Left drag a tip to edit");

    ImGui::SeparatorText("Parameter");
    ImGui::InputDouble("u min", &m_uMin, 0.1, 1.0, "%.3f");
    ImGui::InputDouble("u max", &m_uMax, 0.1, 1.0, "%.3f");
    if (m_uMax <= m_uMin)
        m_uMax = m_uMin + 0.001;
    m_u0 = std::clamp(m_u0, m_uMin, m_uMax);
    ImGui::SliderScalar("u0", ImGuiDataType_Double, &m_u0, &m_uMin, &m_uMax, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    SampleCountSlider(m_sampleCount);

    ImGui::SeparatorText("Horner chain");
    ImGui::Checkbox("Show chain", &m_showChain);
    const int n = static_cast<int>(m_coefficients.size()) - 1;
    ImGui::BeginDisabled(!m_showChain);
    ImGui::SliderInt("Steps shown", &m_stepsShown, 0, n, "%d", ImGuiSliderFlags_AlwaysClamp);
    ImGui::EndDisabled();
    ImGui::TextWrapped("C_n = a_n,  C_k = u0 * C_{k+1} + a_k,  C(u0) = C_0");
    DrawStepsTable(HornerChain());

    ImGui::SeparatorText("Power sum");
    ImGui::Checkbox("Show power sum", &m_showPowerSum);
    ImGui::TextWrapped("C(u0) = a_0 + u0 a_1 + u0^2 a_2 + ... + u0^n a_n, each term drawn tip to tail");

    if (FitViewFooter())
        FitView();
}

void HornerScene::DrawCoefficientView()
{
    m_coefficientView.Begin("##coefficientView");

    // Edit first so this frame draws the updated vectors.
    m_coefficientView.DragPoints(m_coefficients, m_draggedCoefficient);

    constexpr glm::dvec2 kOrigin{0.0, 0.0};
    for (std::size_t i = 0; i < m_coefficients.size(); ++i)
    {
        const bool dragged = static_cast<int>(i) == m_draggedCoefficient;
        const ImU32 color = dragged ? kResultColor : kAddColor;
        m_coefficientView.Arrow(kOrigin, m_coefficients[i], color);
        m_coefficientView.LabeledPoint(m_coefficients[i], IndexedLabel("a", i).c_str(), color, dragged ? 6.0f : 4.0f);
    }
    m_coefficientView.Point(kOrigin, kChainColor, 3.0f);

    m_coefficientView.End();
}

void HornerScene::DrawStepsTable(const std::vector<glm::dvec2>& chain) const
{
    if (!BeginTableWithHeaders("steps", {"k", "a_k", "u0 * C_{k+1}", "C_k"}))
        return;

    // Rows in evaluation order, k = n down to 0; rows past m_stepsShown are dimmed.
    const std::size_t n = chain.size() - 1;
    for (std::size_t s = 0; s <= n; ++s)
    {
        const std::size_t k = n - s;
        const bool shown = !m_showChain || static_cast<int>(s) <= m_stepsShown;
        ImGui::BeginDisabled(!shown);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%zu", k);
        ImGui::TableNextColumn();
        TextPoint(m_coefficients[k]);
        ImGui::TableNextColumn();
        if (k == n)
            ImGui::TextUnformatted("-");
        else
            TextPoint(m_u0 * chain[k + 1]);
        ImGui::TableNextColumn();
        TextPoint(chain[k]);
        ImGui::EndDisabled();
    }
    ImGui::EndTable();
}

void HornerScene::DrawViewport()
{
    m_view.Begin("##horner");

    m_view.Polyline(SampledCurve(), kCurveColor, 2.5f);

    const std::vector<glm::dvec2> chain = HornerChain();
    const std::size_t n = chain.size() - 1;
    if (m_showChain)
    {
        const auto steps = std::min(static_cast<std::size_t>(m_stepsShown), n);
        for (std::size_t s = 0; s <= steps; ++s)
        {
            const std::size_t k = n - s;
            if (k < n)
            {
                // Scale C_{k+1} toward the origin by u0, then translate by a_k.
                const glm::dvec2 previous = chain[k + 1];
                const glm::dvec2 scaled = m_u0 * previous;
                m_view.Line({0.0, 0.0}, previous, kRayColor);
                m_view.Arrow(previous, scaled, kScaleColor);
                m_view.Arrow(scaled, chain[k], kAddColor);
                m_view.Point(scaled, kScaleColor, 3.0f);
            }
            m_view.LabeledPoint(chain[k], IndexedLabel("C", k).c_str(), kChainColor);
        }
    }

    if (m_showPowerSum)
    {
        // The terms u0^i * a[i] tip to tail from the origin; they end at C(u0) too.
        char label[32];
        glm::dvec2 previous{0.0, 0.0};
        const std::vector<glm::dvec2> sums = PowerSum();
        for (std::size_t i = 0; i < sums.size(); ++i)
        {
            m_view.Arrow(previous, sums[i], kPowerColor);
            if (i == 0)
                std::snprintf(label, sizeof(label), "a0");
            else if (i == 1)
                std::snprintf(label, sizeof(label), "u0 a1");
            else
                std::snprintf(label, sizeof(label), "u0^%zu a%zu", i, i);
            m_view.Label(0.5 * (previous + sums[i]), label, kPowerColor);
            previous = sums[i];
        }
    }

    // C_0 = C(u0), the point on the curve.
    m_view.LabeledPoint(chain[0], "C(u0)", kResultColor, 6.0f);

    m_view.End();
}

} // namespace NURBS::Viewer
