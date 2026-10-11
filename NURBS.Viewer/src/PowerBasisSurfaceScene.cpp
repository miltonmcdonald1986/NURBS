#include "PowerBasisSurfaceScene.hpp"

#include "Palette.hpp"
#include "SceneWidgets.hpp"

#include <Graphics/Animation.hpp>
#include <Graphics/Sampling.hpp>
#include <Graphics/SurfaceMesh.hpp>

#include <NURBS/power_basis.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace NURBS::Viewer
{

namespace
{

constexpr int kMaxDegree = 4;

constexpr int kMaxMeshResolution = 256;

constexpr glm::vec3 kSurfaceFrontColor{0.35f, 0.60f, 0.95f};
constexpr glm::vec3 kSurfaceBackColor{0.95f, 0.60f, 0.30f};
// Light and partly transparent, so the isocurves show against the shaded surface.
constexpr ImU32 kIsocurveColor = IM_COL32(230, 230, 240, 150);
// The isocurve S(u, v0) and, brighter, its part from uMin to u0.
constexpr ImU32 kIsoColor = IM_COL32(255, 165, 50, 255);
constexpr ImU32 kProgressColor = IM_COL32(255, 235, 80, 255);
// The parameter domain on the floor.
constexpr ImU32 kDomainOutlineColor = IM_COL32(150, 160, 200, 255);
constexpr ImU32 kDomainGridColor = IM_COL32(100, 110, 145, 200);

// How far below the surface the domain grid sits, as a fraction of the
// surface's largest extent.
constexpr double kFloorGap = 0.15;

// One nonzero coefficient a[i][j].
struct Term
{
    std::size_t i;
    std::size_t j;
    glm::dvec3 a;
};

struct Preset
{
    const char* name;
    std::size_t n;
    std::size_t m;
    double uMin;
    double uMax;
    double vMin;
    double vMax;
    std::vector<Term> terms;
};

const std::array<Preset, 4>& Presets()
{
    // Each is a graph z = f(u, v) over x = u, y = v: a10 = (1, 0, 0) and a01 = (0, 1, 0).
    static const std::array<Preset, 4> presets{{
        {"Plane", 1, 1, 0.0, 1.0, 0.0, 1.0, {{1, 0, {1.0, 0.0, 0.3}}, {0, 1, {0.0, 1.0, 0.2}}}},
        {"Saddle z = uv", 1, 1, -1.0, 1.0, -1.0, 1.0,
         {{1, 0, {1.0, 0.0, 0.0}}, {0, 1, {0.0, 1.0, 0.0}}, {1, 1, {0.0, 0.0, 1.0}}}},
        {"Paraboloid z = u^2 + v^2", 2, 2, -1.0, 1.0, -1.0, 1.0,
         {{1, 0, {1.0, 0.0, 0.0}}, {0, 1, {0.0, 1.0, 0.0}}, {2, 0, {0.0, 0.0, 1.0}}, {0, 2, {0.0, 0.0, 1.0}}}},
        {"Monkey saddle z = u^3 - 3uv^2", 3, 2, -1.0, 1.0, -1.0, 1.0,
         {{1, 0, {1.0, 0.0, 0.0}}, {0, 1, {0.0, 1.0, 0.0}}, {3, 0, {0.0, 0.0, 1.0}}, {1, 2, {0.0, 0.0, -3.0}}}},
    }};
    return presets;
}

// The height of the plane the domain grid is drawn in, a little below the
// surface with the given bounds.
[[nodiscard]] double FloorHeight(const Graphics::Bounds3D& surface)
{
    const glm::dvec3 extent = surface.max - surface.min;
    return surface.min.z - kFloorGap * std::max({extent.x, extent.y, extent.z});
}

} // namespace

PowerBasisSurfaceScene::PowerBasisSurfaceScene()
{
    LoadPreset(1);
}

void PowerBasisSurfaceScene::LoadPreset(std::size_t preset)
{
    const Preset& p = Presets()[preset];
    m_preset = preset;
    m_coefficients.assign(p.n + 1, std::vector<glm::dvec3>(p.m + 1, glm::dvec3{0.0}));
    for (const Term& term : p.terms)
    {
        m_coefficients[term.i][term.j] = term.a;
    }
    m_uMin = p.uMin;
    m_uMax = p.uMax;
    m_vMin = p.vMin;
    m_vMax = p.vMax;
    m_u0 = 0.5 * (m_uMin + m_uMax) + 0.25 * (m_uMax - m_uMin);
    m_v0 = 0.5 * (m_vMin + m_vMax) + 0.25 * (m_vMax - m_vMin);
    m_animateU = false;
    m_animateV = false;
    m_vDirection = 1.0;
    m_meshDirty = true;
    FitView();
}

void PowerBasisSurfaceScene::FitView()
{
    // Fit the domain grid too.
    Graphics::Bounds3D bounds = SurfaceBounds();
    bounds.Extend(glm::dvec3{m_uMin, m_vMin, FloorHeight(bounds)});
    bounds.Extend(glm::dvec3{m_uMax, m_vMax, FloorHeight(bounds)});
    m_view.FitTo(bounds.min, bounds.max);
}

void PowerBasisSurfaceScene::Resize(std::size_t n, std::size_t m)
{
    m_coefficients.resize(n + 1);
    for (std::vector<glm::dvec3>& row : m_coefficients)
    {
        row.resize(m + 1, glm::dvec3{0.0});
    }
    m_meshDirty = true;
}

glm::dvec3 PowerBasisSurfaceScene::Evaluate(double u, double v) const
{
    return NURBS::horner2(m_coefficients, u, v);
}

std::vector<std::vector<glm::dvec3>> PowerBasisSurfaceScene::SampledIsocurves() const
{
    const auto count = static_cast<std::size_t>(m_isocurveCount);
    const auto samples = static_cast<std::size_t>(m_sampleCount);
    std::vector<std::vector<glm::dvec3>> curves;
    curves.reserve(2 * count);
    for (std::size_t k = 0; k < count; ++k)
    {
        const double u = Graphics::ParameterAt(m_uMin, m_uMax, k, count);
        curves.push_back(Graphics::SampleCurve([this, u](double v) { return Evaluate(u, v); }, m_vMin, m_vMax, samples));
    }
    for (std::size_t k = 0; k < count; ++k)
    {
        const double v = Graphics::ParameterAt(m_vMin, m_vMax, k, count);
        curves.push_back(Graphics::SampleCurve([this, v](double u) { return Evaluate(u, v); }, m_uMin, m_uMax, samples));
    }
    return curves;
}

Graphics::Bounds3D PowerBasisSurfaceScene::SurfaceBounds() const
{
    std::optional<Graphics::Bounds3D> bounds;
    for (const std::vector<glm::dvec3>& curve : SampledIsocurves())
    {
        if (!bounds)
            bounds = Graphics::BoundsOf(curve);
        else
            bounds->Extend(curve);
    }
    // There is always at least one isocurve, and each has samples.
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    return *bounds;
}

std::vector<glm::dvec3> PowerBasisSurfaceScene::Isocurve(double v, double uEnd, std::size_t samples) const
{
    return Graphics::SampleCurve([this, v](double u) { return Evaluate(u, v); }, m_uMin, uEnd, samples);
}

void PowerBasisSurfaceScene::Animate()
{
    const double step = SweepStep(m_sweepSeconds);
    if (m_animateU)
        Graphics::AdvanceWrapping(m_u0, m_uMin, m_uMax, step);
    if (m_animateV)
        Graphics::AdvanceBouncing(m_v0, m_vDirection, m_vMin, m_vMax, step);
}

void PowerBasisSurfaceScene::DrawUI()
{
    if (PresetCombo(Presets(), m_preset))
        LoadPreset(m_preset);

    int n = static_cast<int>(m_coefficients.size()) - 1;
    int m = static_cast<int>(m_coefficients[0].size()) - 1;
    const bool nChanged = DegreeSlider("Degree n (u)", n, kMaxDegree);
    const bool mChanged = DegreeSlider("Degree m (v)", m, kMaxDegree);
    if (nChanged || mChanged)
        Resize(static_cast<std::size_t>(n), static_cast<std::size_t>(m));

    if (ImGui::CollapsingHeader("Coefficients a[i][j]"))
    {
        ImGui::TextDisabled("a[i][j] multiplies u^i v^j");
        for (std::size_t i = 0; i < m_coefficients.size(); ++i)
        {
            for (std::size_t j = 0; j < m_coefficients[i].size(); ++j)
            {
                if (ImGui::DragScalarN(Graphics::IndexedLabel("a", i, j).c_str(), ImGuiDataType_Double,
                                       glm::value_ptr(m_coefficients[i][j]), 3, 0.01f, nullptr, nullptr, "%.3f"))
                {
                    m_meshDirty = true;
                }
            }
        }
    }

    ImGui::SeparatorText("Parameters");
    const bool uChanged = InputRange("u", m_uMin, m_uMax);
    const bool vChanged = InputRange("v", m_vMin, m_vMax);
    if (uChanged || vChanged)
        m_meshDirty = true;
    Animate();
    m_u0 = std::clamp(m_u0, m_uMin, m_uMax);
    m_v0 = std::clamp(m_v0, m_vMin, m_vMax);
    Graphics::PlayPauseSlider("u0", "###animateU", "Animate u0: S(u0, v0) moves along the highlighted isocurve", m_u0, m_uMin,
                    m_uMax, m_animateU);
    Graphics::PlayPauseSlider("v0", "###animateV", "Animate v0: the highlighted isocurve moves across the surface", m_v0,
                    m_vMin, m_vMax, m_animateV);
    SweepTimeSlider(m_sweepSeconds);

    ImGui::Text("S(u0, v0) =");
    ImGui::SameLine();
    Graphics::TextPoint(Evaluate(m_u0, m_v0));

    ImGui::SeparatorText("Display");
    ImGui::Checkbox("Shaded surface", &m_showSurface);
    ImGui::Checkbox("Show isocurves", &m_showIsocurves);
    ImGui::SliderInt("Isocurves", &m_isocurveCount, 2, 40, "%d", ImGuiSliderFlags_AlwaysClamp);
}

void PowerBasisSurfaceScene::DrawViewOptions()
{
    ImGui::BeginDisabled(!m_showSurface);
    Graphics::TranslucencyControls(m_translucentSurface, m_surfaceOpacity);
    ImGui::EndDisabled();
    if (ImGui::SliderInt("Mesh resolution", &m_meshResolution, 1, kMaxMeshResolution, "%d",
                         ImGuiSliderFlags_AlwaysClamp))
    {
        m_meshDirty = true;
    }
    ImGui::SetItemTooltip("Grid cells along each of u and v");
    SampleCountSlider(m_sampleCount);
    Graphics::ProjectionCheckbox(m_view.Camera());

    ImGui::Spacing();
    if (Graphics::FitViewButton(Graphics::Viewport3D::kNavigationHint))
        FitView();
}

void PowerBasisSurfaceScene::DrawViewport()
{
    m_view.Begin("##powerBasisSurface");

    if (m_meshDirty)
    {
        const auto resolution = static_cast<std::size_t>(m_meshResolution);
        m_mesh.Upload(Graphics::SampleSurfaceMesh([this](double u, double v) { return Evaluate(u, v); }, m_uMin, m_uMax, m_vMin,
                                        m_vMax, resolution, resolution));
        m_floorZ = FloorHeight(SurfaceBounds());
        m_meshDirty = false;
    }
    if (m_showSurface)
        m_view.DrawMesh(m_mesh, kSurfaceFrontColor, kSurfaceBackColor, m_translucentSurface ? m_surfaceOpacity : 1.0f);

    if (m_showIsocurves)
    {
        for (const std::vector<glm::dvec3>& curve : SampledIsocurves())
        {
            m_view.Polyline(curve, kIsocurveColor, 1.5f);
        }
    }

    DrawFloorGrid();

    // The isocurve S(u, v0), and its part from uMin to u0 by an evenly spaced
    // share of the samples.
    const auto samples = static_cast<std::size_t>(m_sampleCount);
    m_view.Polyline(Isocurve(m_v0, m_uMax, samples), kIsoColor, 2.5f);
    if (m_u0 > m_uMin)
    {
        const double fraction = (m_u0 - m_uMin) / (m_uMax - m_uMin);
        const auto progressSamples =
            std::max<std::size_t>(2, static_cast<std::size_t>(std::lround(fraction * static_cast<double>(samples))));
        m_view.Polyline(Isocurve(m_v0, m_u0, progressSamples), kProgressColor, 4.0f);
    }

    const glm::dvec3 result = Evaluate(m_u0, m_v0);
    m_view.Line({m_u0, m_v0, m_floorZ}, result, kResultColor, 1.5f);
    DrawResultPoint(m_view, result, "S(u0, v0)");

    m_view.End();
}

void PowerBasisSurfaceScene::DrawFloorGrid()
{
    const auto floor = [this](double u, double v) { return glm::dvec3{u, v, m_floorZ}; };

    // Interior lines under the isocurves, at the same u and v values.
    const auto count = static_cast<std::size_t>(m_isocurveCount);
    for (std::size_t k = 1; k + 1 < count; ++k)
    {
        const double u = Graphics::ParameterAt(m_uMin, m_uMax, k, count);
        const double v = Graphics::ParameterAt(m_vMin, m_vMax, k, count);
        m_view.Line(floor(u, m_vMin), floor(u, m_vMax), kDomainGridColor);
        m_view.Line(floor(m_uMin, v), floor(m_uMax, v), kDomainGridColor);
    }

    const std::array<glm::dvec3, 5> outline{floor(m_uMin, m_vMin), floor(m_uMax, m_vMin), floor(m_uMax, m_vMax),
                                            floor(m_uMin, m_vMax), floor(m_uMin, m_vMin)};
    m_view.Polyline(outline, kDomainOutlineColor, 1.5f);
    m_view.Label(floor(m_uMax, m_vMin), "u", kDomainOutlineColor);
    m_view.Label(floor(m_uMin, m_vMax), "v", kDomainOutlineColor);

    // The line v = v0, brighter from uMin to u0, and the point (u0, v0).
    m_view.Line(floor(m_uMin, m_v0), floor(m_uMax, m_v0), kIsoColor, 2.0f);
    m_view.Line(floor(m_uMin, m_v0), floor(m_u0, m_v0), kProgressColor, 3.0f);
    m_view.LabeledPoint(floor(m_u0, m_v0), "(u0, v0)", kResultColor, 5.0f);
}

} // namespace NURBS::Viewer
