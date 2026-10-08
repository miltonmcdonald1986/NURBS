#include "HornerScene.hpp"

#include "CurveSampling.hpp"

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

void TextPoint(glm::dvec2 p)
{
    ImGui::Text("(%.3f, %.3f)", p.x, p.y);
}

} // namespace

HornerScene::HornerScene()
{
    LoadPreset(2);
}

void HornerScene::LoadPreset(int preset)
{
    m_preset = preset;
    m_coefficients = Presets()[static_cast<std::size_t>(preset)].coefficients;
    m_stepsShown = static_cast<int>(m_coefficients.size()) - 1;
    FitView();
}

void HornerScene::FitView()
{
    const std::vector<glm::dvec2> curve = SampledCurve();
    Bounds2D bounds = *BoundsOf(curve);
    if (m_showChain)
    {
        // The chain's scaling rays start at the origin.
        bounds.Extend({0.0, 0.0});
        for (const glm::dvec2& p : HornerChain())
            bounds.Extend(p);
    }
    m_view.FitTo(bounds.min, bounds.max);
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

std::vector<glm::dvec2> HornerScene::SampledCurve() const
{
    return SampleCurve([this](double u) { return NURBS::horner1(m_coefficients, u); }, m_uMin, m_uMax,
                       static_cast<std::size_t>(m_sampleCount));
}

void HornerScene::DrawUI()
{
    const auto& presets = Presets();
    if (ImGui::BeginCombo("Preset", presets[static_cast<std::size_t>(m_preset)].name))
    {
        for (std::size_t i = 0; i < presets.size(); ++i)
        {
            if (ImGui::Selectable(presets[i].name, static_cast<int>(i) == m_preset))
                LoadPreset(static_cast<int>(i));
        }
        ImGui::EndCombo();
    }

    int degree = static_cast<int>(m_coefficients.size()) - 1;
    if (ImGui::SliderInt("Degree n", &degree, 0, kMaxDegree))
    {
        m_coefficients.resize(static_cast<std::size_t>(degree) + 1, glm::dvec2{0.0, 0.0});
        m_stepsShown = std::min(m_stepsShown, degree);
    }

    ImGui::SeparatorText("Coefficients a[i]");
    for (std::size_t i = 0; i < m_coefficients.size(); ++i)
    {
        char label[16];
        std::snprintf(label, sizeof(label), "a%zu", i);
        ImGui::DragScalarN(label, ImGuiDataType_Double, glm::value_ptr(m_coefficients[i]), 2, 0.01f, nullptr, nullptr,
                           "%.3f");
    }

    ImGui::SeparatorText("Parameter");
    ImGui::InputDouble("u min", &m_uMin, 0.1, 1.0, "%.3f");
    ImGui::InputDouble("u max", &m_uMax, 0.1, 1.0, "%.3f");
    if (m_uMax <= m_uMin)
        m_uMax = m_uMin + 0.001;
    m_u0 = std::clamp(m_u0, m_uMin, m_uMax);
    ImGui::SliderScalar("u0", ImGuiDataType_Double, &m_u0, &m_uMin, &m_uMax, "%.3f");
    ImGui::SliderInt("Samples", &m_sampleCount, 2, 1000);

    ImGui::SeparatorText("Horner chain");
    ImGui::Checkbox("Show chain", &m_showChain);
    const int n = static_cast<int>(m_coefficients.size()) - 1;
    ImGui::BeginDisabled(!m_showChain);
    ImGui::SliderInt("Steps shown", &m_stepsShown, 0, n);
    ImGui::EndDisabled();
    ImGui::TextWrapped("C_n = a_n,  C_k = u0 * C_{k+1} + a_k,  C(u0) = C_0");
    DrawStepsTable(HornerChain());

    ImGui::Spacing();
    if (ImGui::Button("Fit view"))
        FitView();
    ImGui::TextDisabled("Right/middle drag: pan   Wheel: zoom");
}

void HornerScene::DrawStepsTable(const std::vector<glm::dvec2>& chain) const
{
    constexpr ImGuiTableFlags kFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable("steps", 4, kFlags))
        return;

    ImGui::TableSetupColumn("k");
    ImGui::TableSetupColumn("a_k");
    ImGui::TableSetupColumn("u0 * C_{k+1}");
    ImGui::TableSetupColumn("C_k");
    ImGui::TableHeadersRow();

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
        char label[16];
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
            m_view.Point(chain[k], kChainColor);
            std::snprintf(label, sizeof(label), "C%zu", k);
            m_view.Label(chain[k], label, kChainColor);
        }
    }

    // C_0 = C(u0), the point on the curve.
    m_view.Point(chain[0], kResultColor, 6.0f);
    m_view.Label(chain[0], "C(u0)", kResultColor);

    m_view.End();
}

} // namespace NURBS::Viewer
