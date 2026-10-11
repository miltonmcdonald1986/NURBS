#include "RationalBezierSurfaceScene.hpp"

#include "ControlNet.hpp"
#include "SceneWidgets.hpp"

#include <Graphics/Sampling.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <span>
#include <vector>

namespace NURBS::Viewer
{

namespace
{

constexpr int kMaxDegree = 6;
constexpr std::size_t kDefaultPreset = 1;

// The weight sliders' range. Weights must stay positive, so the slider is
// clamped even when a value is typed in, and so is the mouse wheel.
constexpr double kMinWeight = 0.01;
constexpr double kMaxWeight = 100.0;
// The factor one wheel notch over a point scales its weight by.
constexpr double kWeightPerNotch = 1.1;

// The polynomial surface's isocurves, each way, and their samples.
constexpr std::size_t kPolynomialIsocurves = 7;
constexpr std::size_t kPolynomialSamples = 32;
constexpr ImU32 kPolynomialColor = IM_COL32(110, 220, 120, 160);

struct Preset
{
    const char* name;
    // P[i][j] and w[i][j]: i runs along u, j along v.
    std::vector<std::vector<glm::dvec3>> controlPoints;
    std::vector<std::vector<double>> weights;
};

// The quarter circle from (1, 0) to (0, 1) as a quadratic rational Bezier
// curve, as in the rational Bezier scene's "Quarter circle" preset.
constexpr std::array<glm::dvec2, 3> kArc{{{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}};
constexpr std::array<double, 3> kArcWeights{1.0, std::numbers::sqrt2 / 2.0, 1.0};

// A quarter cylinder of radius 1 about the y axis and length 1.5: straight
// along y in u, the arc in the xz plane along v, so the normals S_u x S_v
// point outward.
[[nodiscard]] Preset Cylinder()
{
    Preset preset{"Quarter cylinder", {}, {}};
    for (const double y : {0.0, 1.5})
    {
        std::vector<glm::dvec3> row;
        row.reserve(kArc.size());
        for (const glm::dvec2& a : kArc)
            row.emplace_back(a.x, y, a.y);
        preset.controlPoints.push_back(row);
        preset.weights.emplace_back(kArcWeights.begin(), kArcWeights.end());
    }
    return preset;
}

// An eighth of the unit sphere, as a surface of revolution: the arc turns
// about the z axis in u, and as a meridian (radius, height) runs up to the
// pole in v, so the normals S_u x S_v point outward. The weights are the
// products of the two arcs' weights. The last column collapses to the pole.
[[nodiscard]] Preset SphereOctant()
{
    Preset preset{"Sphere octant", {}, {}};
    for (std::size_t i = 0; i < kArc.size(); ++i)
    {
        std::vector<glm::dvec3> row;
        std::vector<double> weights;
        for (std::size_t j = 0; j < kArc.size(); ++j)
        {
            const double radius = kArc[j].x;
            const double height = kArc[j].y;
            row.emplace_back(radius * kArc[i].x, radius * kArc[i].y, height);
            weights.push_back(kArcWeights[i] * kArcWeights[j]);
        }
        preset.controlPoints.push_back(row);
        preset.weights.push_back(weights);
    }
    return preset;
}

// The Bezier surface scene's "Bicubic hill", pulled up toward its four inner
// points by their weights.
[[nodiscard]] Preset WeightedHill()
{
    Preset preset{"Weighted hill", {}, {}};
    for (std::size_t i = 0; i <= 3; ++i)
    {
        std::vector<glm::dvec3> row;
        std::vector<double> weights;
        for (std::size_t j = 0; j <= 3; ++j)
        {
            const bool inner = (i == 1 || i == 2) && (j == 1 || j == 2);
            row.emplace_back(static_cast<double>(i) / 3.0, static_cast<double>(j) / 3.0, inner ? 0.8 : 0.0);
            weights.push_back(inner ? 4.0 : 1.0);
        }
        preset.controlPoints.push_back(row);
        preset.weights.push_back(weights);
    }
    return preset;
}

const std::array<Preset, 3>& Presets()
{
    static const std::array<Preset, 3> presets{Cylinder(), SphereOctant(), WeightedHill()};
    return presets;
}

// A logarithmic weight slider, clamped to [kMinWeight, kMaxWeight]. Returns true when the weight changed.
bool WeightSlider(const char* label, double& weight)
{
    return ImGui::SliderScalar(label, ImGuiDataType_Double, &weight, &kMinWeight, &kMaxWeight, "%.3f",
                               ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
}

} // namespace

RationalBezierSurfaceScene::RationalBezierSurfaceScene()
    : m_surface(Presets()[kDefaultPreset].controlPoints, Presets()[kDefaultPreset].weights)
    , m_preset(kDefaultPreset)
{
    FitView();
}

void RationalBezierSurfaceScene::LoadPreset(std::size_t preset)
{
    m_preset = preset;
    m_surface = NURBS::RationalBezierSurface<glm::dvec3>(Presets()[preset].controlPoints, Presets()[preset].weights);
    m_view.ClearSelection();
    m_view.Invalidate();
    FitView();
}

void RationalBezierSurfaceScene::SetDegrees(int n, int m)
{
    // A RationalBezierSurface's degrees are fixed, so build a new one from the
    // net and weights kept. New points get weight 1.
    const std::size_t rows = m_surface.DegreeU() + 1;
    const std::size_t columns = m_surface.DegreeV() + 1;
    const auto newRows = static_cast<std::size_t>(n) + 1;
    const auto newColumns = static_cast<std::size_t>(m) + 1;
    const std::vector<glm::dvec3> points =
        ResizeControlNet(Flatten(m_surface.ControlPoints()), rows, columns, newRows, newColumns);
    const std::vector<double> weights = ResizeWeights(Flatten(m_surface.Weights()), rows, columns, newRows, newColumns);
    m_surface = NURBS::RationalBezierSurface<glm::dvec3>(ToRows<glm::dvec3>(points, newColumns),
                                                         ToRows<double>(weights, newColumns));
    m_view.ClearSelection();
    m_view.Invalidate();
}

void RationalBezierSurfaceScene::FitView()
{
    m_view.FitTo(Flatten(m_surface.ControlPoints()));
}

void RationalBezierSurfaceScene::DrawUI()
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
    const double u0 = m_view.U0();
    const double v0 = m_view.V0();
    ImGui::Text("S(u0, v0) =");
    ImGui::SameLine();
    Graphics::TextPoint(m_surface.Evaluate(u0, v0));
    ImGui::Text("w(u0, v0) = %.3f", NURBS::deCasteljau2(m_surface.Weights(), u0, v0));
    ImGui::SetItemTooltip("sum B_{i,n}(u0) B_{j,m}(v0) w_{i,j}, the weight S(u0, v0) is divided by");

    m_view.DrawDomainUI(m_surface.DegreeU() + 1, m_surface.DegreeV() + 1, Flatten(m_surface.Weights()));

    ImGui::SeparatorText("Display");
    m_view.DrawDisplayUI();
    ImGui::Checkbox("Show polynomial surface (all w = 1)", &m_showPolynomial);

    ImGui::Spacing();
    ImGui::TextWrapped("S(u, v) = sum B_{i,n}(u) B_{j,m}(v) w_{i,j} P_{i,j} / sum B_{i,n}(u) B_{j,m}(v) w_{i,j}");
    ImGui::TextWrapped("Raising w_{i,j} pulls the surface toward P_{i,j}. RationalBezierSurface::Evaluate runs "
                       "deCasteljau2 (A1.7) on the weighted points w_{i,j} P_{i,j} and on the weights, then divides.");
}

void RationalBezierSurfaceScene::DrawViewOptions()
{
    if (m_view.DrawViewOptions())
        FitView();
}

void RationalBezierSurfaceScene::DrawControlPointsUI()
{
    ImGui::SeparatorText("Control points P[i][j] and weights w[i][j]");
    auto rows = m_surface.ControlPoints();
    auto weights = m_surface.Weights();
    const std::size_t columns = m_surface.DegreeV() + 1;
    bool edited = false;

    if (const int selected = m_view.Selected(); selected >= 0)
    {
        const std::size_t i = static_cast<std::size_t>(selected) / columns;
        const std::size_t j = static_cast<std::size_t>(selected) % columns;
        edited |= ImGui::DragScalarN(Graphics::IndexedLabel("P", i, j).c_str(), ImGuiDataType_Double,
                                     glm::value_ptr(rows[i][j]), 3, 0.01f, nullptr, nullptr, "%.3f");
        edited |= WeightSlider(Graphics::IndexedLabel("w", i, j).c_str(), weights[i][j]);
    }
    else
    {
        ImGui::TextDisabled("Click a point in the view to select it");
    }
    ImGui::TextDisabled("Wheel over a point: scale its weight");

    if (ImGui::CollapsingHeader("All control points and weights"))
    {
        for (std::size_t i = 0; i < rows.size(); ++i)
        {
            for (std::size_t j = 0; j < columns; ++j)
            {
                edited |= ImGui::DragScalarN(Graphics::IndexedLabel("P", i, j).c_str(), ImGuiDataType_Double,
                                             glm::value_ptr(rows[i][j]), 3, 0.01f, nullptr, nullptr, "%.3f");
                edited |= WeightSlider(Graphics::IndexedLabel("w", i, j).c_str(), weights[i][j]);
            }
        }
    }
    if (ImGui::Button("Reset weights"))
    {
        for (auto row : weights)
            std::ranges::fill(row, 1.0);
        edited = true;
    }
    ImGui::SetItemTooltip("Set every weight to 1, which gives the polynomial Bezier surface");
    if (edited)
        m_view.Invalidate();
}

void RationalBezierSurfaceScene::WheelWeight()
{
    const int k = m_view.PointUnderMouse();
    if (k < 0)
        return;
    const float wheel = m_view.TakeMouseWheel();
    if (wheel == 0.0f)
        return;

    const std::size_t columns = m_surface.DegreeV() + 1;
    double& w = m_surface.Weights()[static_cast<std::size_t>(k) / columns][static_cast<std::size_t>(k) % columns];
    w = std::clamp(w * std::pow(kWeightPerNotch, static_cast<double>(wheel)), kMinWeight, kMaxWeight);
    m_view.Invalidate();
}

void RationalBezierSurfaceScene::DrawPolynomialSurface()
{
    const auto rows = m_surface.ControlPoints();
    const auto polynomial = [&rows](double u, double v) { return NURBS::deCasteljau2(rows, u, v); };
    Graphics::Viewport3D& view = m_view.View();
    for (std::size_t k = 0; k < kPolynomialIsocurves; ++k)
    {
        const double t = Graphics::ParameterAt(0.0, 1.0, k, kPolynomialIsocurves);
        view.Polyline(Graphics::SampleCurve([&](double v) { return polynomial(t, v); }, 0.0, 1.0, kPolynomialSamples),
                      kPolynomialColor, 1.5f);
        view.Polyline(Graphics::SampleCurve([&](double u) { return polynomial(u, t); }, 0.0, 1.0, kPolynomialSamples),
                      kPolynomialColor, 1.5f);
    }
}

void RationalBezierSurfaceScene::DrawViewport()
{
    // Edit a flat copy of the net, so this frame draws the moved point.
    std::vector<glm::dvec3> points = Flatten(m_surface.ControlPoints());
    if (m_view.Begin("##rationalBezierSurface", points))
        Unflatten(std::span<const glm::dvec3>{points}, m_surface.ControlPoints());
    WheelWeight();

    if (m_showPolynomial)
        DrawPolynomialSurface();
    const std::vector<double> weights = Flatten(m_surface.Weights());
    m_view.Draw([this](double u, double v) { return m_surface.Evaluate(u, v); }, points, m_surface.DegreeV() + 1,
                weights);
    m_view.End();
}

} // namespace NURBS::Viewer
