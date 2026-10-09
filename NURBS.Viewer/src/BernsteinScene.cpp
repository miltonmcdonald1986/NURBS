#include "BernsteinScene.hpp"

#include "CurveSampling.hpp"
#include "ImGuiHelpers.hpp"

#include <NURBS/bernstein.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace NURBS::Viewer
{

namespace
{

constexpr int kMaxDegree = 10;

// Bounds for the u0 slider, which takes them by address.
constexpr double kZero = 0.0;
constexpr double kOne = 1.0;

constexpr ImU32 kCursorColor = IM_COL32(200, 200, 210, 160);
constexpr ImU32 kTextColor = IM_COL32(230, 230, 230, 255);

// The partition-of-unity bar, just right of the plot.
constexpr double kBarLeft = 1.08;
constexpr double kBarRight = 1.14;

} // namespace

BernsteinScene::BernsteinScene()
{
    FitView();
}

void BernsteinScene::FitView()
{
    m_view.FitTo({-0.05, -0.05}, {m_showPartition ? 1.2 : 1.05, 1.05});
}

ImU32 BernsteinScene::ColorOf(std::size_t i, float alpha) const
{
    const float hue = static_cast<float>(i) / static_cast<float>(m_degree + 1);
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    ImGui::ColorConvertHSVtoRGB(hue, 0.65f, 1.0f, r, g, b);
    return ImGui::ColorConvertFloat4ToU32(ImVec4(r, g, b, alpha));
}

bool BernsteinScene::IsDimmed(std::size_t i) const
{
    return m_highlight && static_cast<int>(i) != m_highlighted;
}

std::vector<std::vector<glm::dvec2>> BernsteinScene::SampledBasis() const
{
    // One AllBernstein call per sample gives a point on every graph.
    const auto n = static_cast<std::size_t>(m_degree);
    const auto count = static_cast<std::size_t>(m_sampleCount);
    std::vector<std::vector<glm::dvec2>> basis(n + 1);
    for (auto& graph : basis)
    {
        graph.reserve(count);
    }

    const double last = static_cast<double>(count - 1);
    for (std::size_t s = 0; s < count; ++s)
    {
        const double u = static_cast<double>(s) / last;
        const std::vector<double> values = NURBS::AllBernstein(n, u);
        for (std::size_t i = 0; i <= n; ++i)
        {
            basis[i].emplace_back(u, values[i]);
        }
    }
    return basis;
}

void BernsteinScene::DrawUI()
{
    if (ImGui::SliderInt("Degree n", &m_degree, 0, kMaxDegree, "%d", ImGuiSliderFlags_AlwaysClamp))
        m_highlighted = std::min(m_highlighted, m_degree);
    ImGui::SliderScalar("u0", ImGuiDataType_Double, &m_u0, &kZero, &kOne, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderInt("Samples", &m_sampleCount, 2, 1000, "%d", ImGuiSliderFlags_AlwaysClamp);

    ImGui::SeparatorText("Display");
    ImGui::Checkbox("Highlight one (A1.2)", &m_highlight);
    ImGui::BeginDisabled(!m_highlight);
    ImGui::SliderInt("i", &m_highlighted, 0, m_degree, "%d", ImGuiSliderFlags_AlwaysClamp);
    ImGui::EndDisabled();
    if (ImGui::Checkbox("Show partition of unity", &m_showPartition))
        FitView();

    ImGui::SeparatorText("Values at u0");
    DrawValuesTable(NURBS::AllBernstein(static_cast<std::size_t>(m_degree), m_u0));

    ImGui::Spacing();
    ImGui::TextWrapped("B_{i,n}(u) = n! / (i! (n-i)!) u^i (1-u)^(n-i)");
    ImGui::TextWrapped("B_{i,n}(u) = (1-u) B_{i,n-1}(u) + u B_{i-1,n-1}(u)");
    ImGui::TextWrapped("A1.2 runs the recurrence for one i; A1.3 runs it for all i at once.");

    if (FitViewFooter())
        FitView();
}

void BernsteinScene::DrawValuesTable(const std::vector<double>& values) const
{
    if (!BeginTableWithHeaders("values", {"i", "A1.3", "A1.2", "|diff|"}))
        return;

    const auto n = static_cast<std::size_t>(m_degree);
    double sum = 0.0;
    for (std::size_t i = 0; i <= n; ++i)
    {
        const double single = NURBS::Bernstein(i, n, m_u0);
        sum += values[i];
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ColorOf(i)), "%zu", i);
        ImGui::TableNextColumn();
        ImGui::Text("%.6f", values[i]);
        ImGui::TableNextColumn();
        ImGui::Text("%.6f", single);
        ImGui::TableNextColumn();
        ImGui::Text("%.1e", std::abs(values[i] - single));
    }

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("sum");
    ImGui::TableNextColumn();
    ImGui::Text("%.6f", sum);
    ImGui::EndTable();
}

void BernsteinScene::DrawViewport()
{
    m_view.Begin("##bernstein");

    const auto n = static_cast<std::size_t>(m_degree);
    const std::vector<std::vector<glm::dvec2>> basis = SampledBasis();
    for (std::size_t i = 0; i <= n; ++i)
    {
        m_view.Polyline(basis[i], ColorOf(i, IsDimmed(i) ? 0.3f : 1.0f), 2.0f);
    }

    if (m_highlight)
    {
        // The same graph from Bernstein (A1.2); it lies exactly over the A1.3 one.
        const auto i = static_cast<std::size_t>(m_highlighted);
        const std::vector<glm::dvec2> single = SampleCurve(
            [i, n](double u) { return glm::dvec2{u, NURBS::Bernstein(i, n, u)}; }, 0.0, 1.0,
            static_cast<std::size_t>(m_sampleCount));
        m_view.Polyline(single, ColorOf(i), 4.0f);
    }

    // The evaluation line at u0 and the value of every basis function on it.
    m_view.Line({m_u0, 0.0}, {m_u0, 1.0}, kCursorColor);
    const std::vector<double> values = NURBS::AllBernstein(n, m_u0);
    for (std::size_t i = 0; i <= n; ++i)
    {
        if (IsDimmed(i))
            continue;
        m_view.LabeledPoint({m_u0, values[i]}, IndexedLabel("B", i).c_str(), ColorOf(i));
    }

    if (m_showPartition)
    {
        // The values stacked bottom to top; they always reach 1.
        double bottom = 0.0;
        for (std::size_t i = 0; i <= n; ++i)
        {
            const double top = bottom + values[i];
            m_view.Rect({kBarLeft, bottom}, {kBarRight, top}, ColorOf(i));
            bottom = top;
        }
        m_view.Line({kBarLeft, 1.0}, {kBarRight, 1.0}, kTextColor);
        char label[32];
        std::snprintf(label, sizeof(label), "sum = %.3f", bottom);
        m_view.Label({kBarLeft, bottom}, label, kTextColor);
    }

    m_view.End();
}

} // namespace NURBS::Viewer
