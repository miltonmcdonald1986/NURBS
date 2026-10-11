#include "BezierSurfaceScene.hpp"

#include "ControlNet.hpp"
#include "SceneWidgets.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <array>
#include <span>
#include <vector>

namespace NURBS::Viewer
{

namespace
{

constexpr int kMaxDegree = 6;
constexpr std::size_t kDefaultPreset = 1;

struct Preset
{
    const char* name;
    // P[i][j]: i runs along u, j along v.
    std::vector<std::vector<glm::dvec3>> controlPoints;
};

// A (n + 1) x (m + 1) net over the unit square, P[i][j] = (i / n, j / m, height(i, j)).
template <typename Height>
[[nodiscard]] std::vector<std::vector<glm::dvec3>> GridNet(std::size_t n, std::size_t m, Height height)
{
    std::vector<std::vector<glm::dvec3>> net(n + 1);
    for (std::size_t i = 0; i <= n; ++i)
    {
        for (std::size_t j = 0; j <= m; ++j)
        {
            net[i].emplace_back(static_cast<double>(i) / static_cast<double>(n),
                                static_cast<double>(j) / static_cast<double>(m), height(i, j));
        }
    }
    return net;
}

const std::array<Preset, 4>& Presets()
{
    static const std::array<Preset, 4> presets{{
        // Opposite corners raised: a hyperbolic paraboloid, ruled by straight isocurves.
        {"Bilinear saddle", {{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.5}}, {{1.0, 0.0, 0.5}, {1.0, 1.0, 0.0}}}},
        // The four inner points raised.
        {"Bicubic hill", GridNet(3, 3, [](std::size_t i, std::size_t j) {
             return (i == 1 || i == 2) && (j == 1 || j == 2) ? 0.8 : 0.0;
         })},
        // The center lowered below a level rim.
        {"Biquadratic bowl", GridNet(2, 2, [](std::size_t i, std::size_t j) { return i == 1 && j == 1 ? -0.8 : 0.0; })},
        // Heights alternating like a checkerboard.
        {"Egg crate", GridNet(4, 4, [](std::size_t i, std::size_t j) { return (i + j) % 2 == 0 ? 0.25 : -0.25; })},
    }};
    return presets;
}

} // namespace

BezierSurfaceScene::BezierSurfaceScene()
    : m_surface(Presets()[kDefaultPreset].controlPoints)
    , m_preset(kDefaultPreset)
{
    FitView();
}

void BezierSurfaceScene::LoadPreset(std::size_t preset)
{
    m_preset = preset;
    m_surface = NURBS::BezierSurface<glm::dvec3>(Presets()[preset].controlPoints);
    m_view.ClearSelection();
    m_view.Invalidate();
    FitView();
}

void BezierSurfaceScene::SetDegrees(int n, int m)
{
    // A BezierSurface's degrees are fixed, so build a new one from the net kept.
    const std::vector<glm::dvec3> points = Flatten(m_surface.ControlPoints());
    const std::size_t columns = m_surface.DegreeV() + 1;
    const auto newColumns = static_cast<std::size_t>(m) + 1;
    const std::vector<glm::dvec3> resized =
        ResizeControlNet(points, m_surface.DegreeU() + 1, columns, static_cast<std::size_t>(n) + 1, newColumns);
    m_surface = NURBS::BezierSurface<glm::dvec3>(ToRows<glm::dvec3>(resized, newColumns));
    m_view.ClearSelection();
    m_view.Invalidate();
}

void BezierSurfaceScene::FitView()
{
    m_view.FitTo(Flatten(m_surface.ControlPoints()));
}

void BezierSurfaceScene::DrawUI()
{
    if (PresetCombo(Presets(), m_preset))
        LoadPreset(m_preset);

    int n = static_cast<int>(m_surface.DegreeU());
    int m = static_cast<int>(m_surface.DegreeV());
    const bool nChanged = DegreeSlider("Degree n (u)", n, kMaxDegree);
    const bool mChanged = DegreeSlider("Degree m (v)", m, kMaxDegree);
    if (nChanged || mChanged)
        SetDegrees(n, m);

    DrawControlPointsUI();

    ImGui::SeparatorText("Parameters");
    m_view.DrawParameterUI();
    ImGui::Text("S(u0, v0) =");
    ImGui::SameLine();
    Graphics::TextPoint(m_surface.Evaluate(m_view.U0(), m_view.V0()));

    m_view.DrawDomainUI(m_surface.DegreeU() + 1, m_surface.DegreeV() + 1);

    ImGui::SeparatorText("Display");
    m_view.DrawDisplayUI();

    ImGui::Spacing();
    ImGui::TextWrapped("S(u, v) = sum B_{i,n}(u) B_{j,m}(v) P_{i,j},  i = 0..n, j = 0..m");
    ImGui::TextWrapped("A1.7 (deCasteljau2, used by BezierSurface::Evaluate) runs de Casteljau (A1.5) along v on "
                       "each row of the net, then once along u on the resulting points.");
}

void BezierSurfaceScene::DrawViewOptions()
{
    if (m_view.DrawViewOptions())
        FitView();
}

void BezierSurfaceScene::DrawControlPointsUI()
{
    ImGui::SeparatorText("Control points P[i][j]");
    auto rows = m_surface.ControlPoints();
    const std::size_t columns = m_surface.DegreeV() + 1;
    bool edited = false;

    if (const int selected = m_view.Selected(); selected >= 0)
    {
        const auto k = static_cast<std::size_t>(selected);
        glm::dvec3& p = rows[k / columns][k % columns];
        edited |= ImGui::DragScalarN(Graphics::IndexedLabel("P", k / columns, k % columns).c_str(),
                                     ImGuiDataType_Double, glm::value_ptr(p), 3, 0.01f, nullptr, nullptr, "%.3f");
    }
    else
    {
        ImGui::TextDisabled("Click a point in the view to select it");
    }

    if (ImGui::CollapsingHeader("All control points"))
    {
        for (std::size_t i = 0; i < rows.size(); ++i)
        {
            for (std::size_t j = 0; j < columns; ++j)
            {
                edited |= ImGui::DragScalarN(Graphics::IndexedLabel("P", i, j).c_str(), ImGuiDataType_Double,
                                             glm::value_ptr(rows[i][j]), 3, 0.01f, nullptr, nullptr, "%.3f");
            }
        }
    }
    if (edited)
        m_view.Invalidate();
}

void BezierSurfaceScene::DrawViewport()
{
    // Edit a flat copy of the net, so this frame draws the moved point.
    std::vector<glm::dvec3> points = Flatten(m_surface.ControlPoints());
    if (m_view.Begin("##bezierSurface", points))
        Unflatten(std::span<const glm::dvec3>{points}, m_surface.ControlPoints());

    m_view.Draw([this](double u, double v) { return m_surface.Evaluate(u, v); }, points, m_surface.DegreeV() + 1);
    m_view.End();
}

} // namespace NURBS::Viewer
