#include "PowerBasisSurfaceScene.hpp"

#include "CurveSampling.hpp"
#include "ImGuiHelpers.hpp"
#include "SurfaceSampling.hpp"

#include <NURBS/power_basis.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
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
constexpr ImU32 kResultColor = IM_COL32(255, 90, 90, 255);

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

// Keeps max above min after either was edited.
void KeepOrdered(double min, double& max)
{
    if (max <= min)
        max = min + 0.001;
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
    m_meshDirty = true;
    FitView();
}

void PowerBasisSurfaceScene::FitView()
{
    std::optional<Bounds3D> bounds;
    for (const std::vector<glm::dvec3>& curve : SampledIsocurves())
    {
        const std::optional<Bounds3D> curveBounds = BoundsOf(curve);
        if (!bounds)
        {
            bounds = curveBounds;
        }
        else
        {
            bounds->Extend(curveBounds->min);
            bounds->Extend(curveBounds->max);
        }
    }
    m_view.FitTo(bounds->min, bounds->max);
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
    const double last = static_cast<double>(count - 1);

    std::vector<std::vector<glm::dvec3>> curves;
    curves.reserve(2 * count);
    for (std::size_t k = 0; k < count; ++k)
    {
        const double u = m_uMin + (m_uMax - m_uMin) * (static_cast<double>(k) / last);
        curves.push_back(SampleCurve([this, u](double v) { return Evaluate(u, v); }, m_vMin, m_vMax, samples));
    }
    for (std::size_t k = 0; k < count; ++k)
    {
        const double v = m_vMin + (m_vMax - m_vMin) * (static_cast<double>(k) / last);
        curves.push_back(SampleCurve([this, v](double u) { return Evaluate(u, v); }, m_uMin, m_uMax, samples));
    }
    return curves;
}

void PowerBasisSurfaceScene::DrawUI()
{
    if (PresetCombo(Presets(), m_preset, [](const Preset& preset) { return preset.name; }))
        LoadPreset(m_preset);

    int n = static_cast<int>(m_coefficients.size()) - 1;
    int m = static_cast<int>(m_coefficients[0].size()) - 1;
    const bool nChanged = ImGui::SliderInt("Degree n (u)", &n, 0, kMaxDegree, "%d", ImGuiSliderFlags_AlwaysClamp);
    const bool mChanged = ImGui::SliderInt("Degree m (v)", &m, 0, kMaxDegree, "%d", ImGuiSliderFlags_AlwaysClamp);
    if (nChanged || mChanged)
        Resize(static_cast<std::size_t>(n), static_cast<std::size_t>(m));

    if (ImGui::CollapsingHeader("Coefficients a[i][j]"))
    {
        ImGui::TextDisabled("a[i][j] multiplies u^i v^j");
        for (std::size_t i = 0; i < m_coefficients.size(); ++i)
        {
            for (std::size_t j = 0; j < m_coefficients[i].size(); ++j)
            {
                if (ImGui::DragScalarN(IndexedLabel("a", i, j).c_str(), ImGuiDataType_Double,
                                       glm::value_ptr(m_coefficients[i][j]), 3, 0.01f, nullptr, nullptr, "%.3f"))
                {
                    m_meshDirty = true;
                }
            }
        }
    }

    ImGui::SeparatorText("Parameters");
    bool domainChanged = ImGui::InputDouble("u min", &m_uMin, 0.1, 1.0, "%.3f");
    domainChanged |= ImGui::InputDouble("u max", &m_uMax, 0.1, 1.0, "%.3f");
    KeepOrdered(m_uMin, m_uMax);
    domainChanged |= ImGui::InputDouble("v min", &m_vMin, 0.1, 1.0, "%.3f");
    domainChanged |= ImGui::InputDouble("v max", &m_vMax, 0.1, 1.0, "%.3f");
    KeepOrdered(m_vMin, m_vMax);
    if (domainChanged)
        m_meshDirty = true;
    m_u0 = std::clamp(m_u0, m_uMin, m_uMax);
    m_v0 = std::clamp(m_v0, m_vMin, m_vMax);
    ImGui::SliderScalar("u0", ImGuiDataType_Double, &m_u0, &m_uMin, &m_uMax, "%.3f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderScalar("v0", ImGuiDataType_Double, &m_v0, &m_vMin, &m_vMax, "%.3f", ImGuiSliderFlags_AlwaysClamp);

    ImGui::Text("S(u0, v0) =");
    ImGui::SameLine();
    TextPoint(Evaluate(m_u0, m_v0));

    ImGui::SeparatorText("Display");
    ImGui::Checkbox("Shaded surface", &m_showSurface);
    if (ImGui::SliderInt("Mesh resolution", &m_meshResolution, 1, kMaxMeshResolution, "%d",
                         ImGuiSliderFlags_AlwaysClamp))
    {
        m_meshDirty = true;
    }
    ImGui::SetItemTooltip("Grid cells along each of u and v");
    ImGui::Checkbox("Show isocurves", &m_showIsocurves);
    ImGui::SliderInt("Isocurves", &m_isocurveCount, 2, 40, "%d", ImGuiSliderFlags_AlwaysClamp);
    SampleCountSlider(m_sampleCount);
    bool orthographic = m_view.Camera().IsOrthographic();
    if (ImGui::Checkbox("Orthographic", &orthographic))
        m_view.Camera().SetOrthographic(orthographic);

    if (FitViewFooter("Left drag: orbit   Right/middle drag: pan   Wheel: zoom"))
        FitView();
}

void PowerBasisSurfaceScene::DrawViewport()
{
    m_view.Begin("##powerBasisSurface");

    if (m_showSurface)
    {
        if (m_meshDirty)
        {
            const auto resolution = static_cast<std::size_t>(m_meshResolution);
            m_mesh.Upload(SampleSurfaceMesh([this](double u, double v) { return Evaluate(u, v); }, m_uMin, m_uMax,
                                            m_vMin, m_vMax, resolution, resolution));
            m_meshDirty = false;
        }
        m_view.DrawMesh(m_mesh, kSurfaceFrontColor, kSurfaceBackColor);
    }

    if (m_showIsocurves)
    {
        for (const std::vector<glm::dvec3>& curve : SampledIsocurves())
        {
            m_view.Polyline(curve, kIsocurveColor, 1.5f);
        }
    }
    m_view.LabeledPoint(Evaluate(m_u0, m_v0), "S(u0, v0)", kResultColor, 6.0f);

    m_view.End();
}

} // namespace NURBS::Viewer
