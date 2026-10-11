#include "SurfaceView.hpp"

#include "Palette.hpp"
#include "SceneWidgets.hpp"

#include <Graphics/Animation.hpp>
#include <Graphics/Bounds.hpp>
#include <Graphics/Sampling.hpp>
#include <Graphics/SurfaceMesh.hpp>
#include <Graphics/Theme.hpp>

#include <NURBS/bernstein.hpp>

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <format>
#include <optional>
#include <string>

namespace NURBS::Viewer
{

namespace
{

constexpr int kMaxMeshResolution = 256;
// The mesh resolution and isocurve samples while a point is dragged, so the
// surface keeps up with the mouse.
constexpr int kDragMeshResolution = 24;
constexpr int kDragSampleCount = 24;

constexpr glm::vec3 kSurfaceFrontColor{0.35f, 0.60f, 0.95f};
constexpr glm::vec3 kSurfaceBackColor{0.95f, 0.60f, 0.30f};
// Light and partly transparent, so the isocurves show against the shaded surface.
constexpr ImU32 kIsocurveColor = IM_COL32(230, 230, 240, 150);
// The isocurve S(u, v0) and, brighter, its part from 0 to u0.
constexpr ImU32 kIsoColor = IM_COL32(255, 165, 50, 255);
constexpr ImU32 kProgressColor = IM_COL32(255, 235, 80, 255);

constexpr ImU32 kNetColor = IM_COL32(150, 150, 170, 220);
constexpr ImU32 kControlPointColor = IM_COL32(240, 180, 60, 255);
constexpr ImU32 kHoveredColor = IM_COL32(255, 230, 160, 255);
constexpr ImU32 kSelectedColor = IM_COL32(120, 230, 255, 255);
constexpr ImU32 kDraggedColor = IM_COL32(255, 255, 255, 255);
// Where the dragged point was grabbed, and its line down to the z = 0 grid.
constexpr ImU32 kGhostColor = IM_COL32(170, 170, 180, 150);
constexpr ImU32 kDropColor = IM_COL32(200, 200, 210, 170);

// The parameter domain in the inset, as in the power basis surface scene's floor grid.
constexpr ImU32 kDomainOutlineColor = IM_COL32(150, 160, 200, 255);
constexpr ImU32 kDomainGridColor = IM_COL32(100, 110, 145, 200);
// The control points' marks at their basis functions' peaks.
constexpr ImU32 kPeakColor = IM_COL32(240, 180, 60, 150);
// The inset's margin around the domain, in domain units.
constexpr double kDomainMargin = 0.1;
// How near the mouse, in pixels, a mark is hovered.
constexpr float kMarkPickPixels = 7.0f;
// Cells along each side of the influence shading, and its color, fully
// opaque at the peak and fading out with the basis function.
constexpr std::size_t kInfluenceCells = 48;
constexpr ImU32 kInfluenceColor = IM_COL32(255, 120, 40, 0);
constexpr double kInfluenceMaxAlpha = 220.0;

// Point sizes follow the weight's square root, within these factors.
constexpr double kMinWeightScale = 0.6;
constexpr double kMaxWeightScale = 2.0;
// The constraint guides' half-length, as a share of the net's extent.
constexpr double kGuideShare = 0.3;

[[nodiscard]] float WeightScale(double weight)
{
    return static_cast<float>(std::clamp(std::sqrt(weight), kMinWeightScale, kMaxWeightScale));
}

// "P" followed by i and j and, if given, the weight, e.g. "P1,2 w=0.71".
[[nodiscard]] Graphics::Label16 ControlPointLabel(std::size_t i, std::size_t j, const double* weight)
{
    if (weight == nullptr)
        return Graphics::IndexedLabel("P", i, j);
    return Graphics::FormatLabel("P{},{} w={:.2f}", i, j, *weight);
}

// Where B_{i,degree} peaks: at i / degree, or anywhere (say the middle) for degree 0.
[[nodiscard]] double PeakParameter(std::size_t i, std::size_t degree)
{
    return degree == 0 ? 0.5 : static_cast<double>(i) / static_cast<double>(degree);
}

void FitDomain(Graphics::Viewport2D& view)
{
    view.FitTo({-kDomainMargin, -kDomainMargin}, {1.0 + kDomainMargin, 1.0 + kDomainMargin});
}

} // namespace

SurfaceView::SurfaceView()
{
    FitDomain(m_domainView);
}

void SurfaceView::ClearSelection()
{
    m_drag = {};
    m_selected = -1;
}

void SurfaceView::FitTo(std::span<const glm::dvec3> points)
{
    assert(!points.empty());

    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    const Graphics::Bounds3D bounds = *Graphics::BoundsOf(points);
    m_extent = std::max(glm::length(bounds.max - bounds.min), 1e-3);
    m_view.FitTo(bounds.min, bounds.max);
    FitDomain(m_domainView);
}

int SurfaceView::PointUnderMouse() const
{
    return Dragging() ? m_drag.active : m_drag.hovered;
}

int SurfaceView::FocusPoint() const
{
    if (const int k = PointUnderMouse(); k >= 0)
        return k;
    if (m_domainHovered >= 0)
        return m_domainHovered;
    return m_selected;
}

void SurfaceView::DrawDomainUI(std::size_t rows, std::size_t columns, std::span<const double> weights)
{
    assert(rows > 0 && columns > 0);
    assert(weights.empty() || weights.size() == rows * columns);

    if (!ImGui::CollapsingHeader("Domain", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Not drawn, so nothing in it is under the mouse.
        m_domainHovered = -1;
        m_domainDragged = -1;
        return;
    }

    ImGui::Checkbox("Shade influence", &m_showInfluence);
    ImGui::SetItemTooltip("Shade the domain by the basis function of the point under the mouse (else the selected "
                          "one): where it is large, dragging that point moves the surface most");

    // A square canvas the width of the panel.
    const float side = ImGui::GetContentRegionAvail().x;
    if (ImGui::BeginChild("##domain", ImVec2(side, side), ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
    {
        DrawDomain(rows, columns, weights);
    }
    ImGui::EndChild();

    if (m_influencePoint >= 0)
    {
        const auto k = static_cast<std::size_t>(m_influencePoint);
        ImGui::TextDisabled("Shading %s of P%zu,%zu, peak %.3f", weights.empty() ? "B_{i,n}(u) B_{j,m}(v)" : "R_{i,j}(u, v)",
                            k / columns, k % columns, m_influencePeak);
    }
    else
    {
        ImGui::TextDisabled("Hover a point in either view to shade its influence");
    }
    ImGui::TextDisabled("Left drag (u0, v0) to move it");
}

void SurfaceView::DrawDomain(std::size_t rows, std::size_t columns, std::span<const double> weights)
{
    m_domainView.Begin("##domainView");

    // Edit first so this frame draws the moved point. Dragging stops the animations.
    std::array<glm::dvec2, 1> parameters{{{m_u0, m_v0}}};
    if (m_domainView.DragPoints(parameters, m_domainDragged))
    {
        m_u0 = std::clamp(parameters[0].x, 0.0, 1.0);
        m_v0 = std::clamp(parameters[0].y, 0.0, 1.0);
        m_animateU = false;
        m_animateV = false;
    }

    // Each control point's mark, where its basis function peaks.
    const auto peak = [rows, columns](std::size_t k) {
        return glm::dvec2{PeakParameter(k / columns, rows - 1), PeakParameter(k % columns, columns - 1)};
    };
    const std::size_t count = rows * columns;

    m_domainHovered = -1;
    if (m_domainView.IsHovered() && m_domainDragged < 0)
    {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        float nearest = kMarkPickPixels;
        for (std::size_t k = 0; k < count; ++k)
        {
            const ImVec2 s = m_domainView.ToScreen(peak(k));
            const float pixels = std::hypot(s.x - mouse.x, s.y - mouse.y);
            if (pixels <= nearest)
            {
                nearest = pixels;
                m_domainHovered = static_cast<int>(k);
            }
        }
    }

    const int focus = FocusPoint();
    m_influencePoint = -1;
    if (m_showInfluence && focus >= 0 && static_cast<std::size_t>(focus) < count)
    {
        m_influencePeak = DrawInfluence(static_cast<std::size_t>(focus), rows, columns, weights);
        m_influencePoint = focus;
    }

    // Interior lines at the isocurves' parameters, then the outline.
    const auto lines = static_cast<std::size_t>(m_isocurveCount);
    for (std::size_t k = 1; k + 1 < lines; ++k)
    {
        const double t = Graphics::ParameterAt(0.0, 1.0, k, lines);
        m_domainView.Line({t, 0.0}, {t, 1.0}, kDomainGridColor);
        m_domainView.Line({0.0, t}, {1.0, t}, kDomainGridColor);
    }
    const std::array<glm::dvec2, 5> outline{{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}}};
    m_domainView.Polyline(outline, kDomainOutlineColor, 1.5f);
    m_domainView.Label({1.0, 0.0}, "u", kDomainOutlineColor);
    m_domainView.Label({0.0, 1.0}, "v", kDomainOutlineColor);

    // The line v = v0, brighter from 0 to u0, as the highlighted isocurve S(u, v0).
    m_domainView.Line({0.0, m_v0}, {1.0, m_v0}, kIsoColor, 2.0f);
    m_domainView.Line({0.0, m_v0}, {m_u0, m_v0}, kProgressColor, 3.0f);

    for (std::size_t k = 0; k < count; ++k)
    {
        if (static_cast<int>(k) != focus)
            m_domainView.Point(peak(k), kPeakColor, 3.0f);
    }
    if (focus >= 0 && static_cast<std::size_t>(focus) < count)
    {
        const auto k = static_cast<std::size_t>(focus);
        m_domainView.LabeledPoint(peak(k), Graphics::IndexedLabel("P", k / columns, k % columns).c_str(), kHoveredColor,
                                  kDraggedPointRadius);
    }

    m_domainView.LabeledPoint({m_u0, m_v0}, "(u0, v0)", kResultColor, kResultRadius);

    m_domainView.End();
}

double SurfaceView::DrawInfluence(std::size_t k, std::size_t rows, std::size_t columns,
                                  std::span<const double> weights) const
{
    const std::size_t n = rows - 1;
    const std::size_t m = columns - 1;
    const std::size_t i = k / columns;
    const std::size_t j = k % columns;

    // The basis functions at the cells' centers, along u and along v.
    std::vector<std::vector<double>> bu;
    std::vector<std::vector<double>> bv;
    for (std::size_t c = 0; c < kInfluenceCells; ++c)
    {
        const double t = (static_cast<double>(c) + 0.5) / static_cast<double>(kInfluenceCells);
        bu.push_back(NURBS::AllBernstein(n, t));
        bv.push_back(NURBS::AllBernstein(m, t));
    }

    // B_{i,n}(u) B_{j,m}(v), or for a rational surface R_{i,j}(u, v), cell by cell.
    std::vector<double> values(kInfluenceCells * kInfluenceCells);
    double peak = 0.0;
    for (std::size_t a = 0; a < kInfluenceCells; ++a)
    {
        for (std::size_t b = 0; b < kInfluenceCells; ++b)
        {
            double value = bu[a][i] * bv[b][j];
            if (!weights.empty())
            {
                double denominator = 0.0;
                for (std::size_t p = 0; p <= n; ++p)
                {
                    for (std::size_t q = 0; q <= m; ++q)
                        denominator += bu[a][p] * bv[b][q] * weights[p * columns + q];
                }
                value *= weights[k] / denominator;
            }
            values[a * kInfluenceCells + b] = value;
            peak = std::max(peak, value);
        }
    }
    if (peak <= 0.0)
        return 0.0;

    const double cell = 1.0 / static_cast<double>(kInfluenceCells);
    for (std::size_t a = 0; a < kInfluenceCells; ++a)
    {
        for (std::size_t b = 0; b < kInfluenceCells; ++b)
        {
            const auto alpha =
                static_cast<ImU32>(std::lround(kInfluenceMaxAlpha * values[a * kInfluenceCells + b] / peak));
            const glm::dvec2 corner{static_cast<double>(a) * cell, static_cast<double>(b) * cell};
            m_domainView.Rect(corner, corner + glm::dvec2{cell, cell}, kInfluenceColor | (alpha << IM_COL32_A_SHIFT));
        }
    }
    return peak;
}

void SurfaceView::DrawParameterUI()
{
    const double step = SweepStep(m_sweepSeconds);
    if (m_animateU)
        Graphics::AdvanceWrapping(m_u0, 0.0, 1.0, step);
    if (m_animateV)
        Graphics::AdvanceBouncing(m_v0, m_vDirection, 0.0, 1.0, step);
    constexpr double kMin = 0.0;
    constexpr double kMax = 1.0;
    Graphics::PlayPauseSlider("u0", "###animateU", "Animate u0: S(u0, v0) moves along the highlighted isocurve", m_u0,
                              kMin, kMax, m_animateU);
    Graphics::PlayPauseSlider("v0", "###animateV", "Animate v0: the highlighted isocurve moves across the surface",
                              m_v0, kMin, kMax, m_animateV);
    SweepTimeSlider(m_sweepSeconds);
}

void SurfaceView::DrawDisplayUI()
{
    ImGui::Checkbox("Shaded surface", &m_showSurface);
    ImGui::BeginDisabled(!m_showSurface);
    ImGui::Checkbox("Translucent", &m_translucentSurface);
    ImGui::SetItemTooltip("Show what lies behind the surface through it");
    if (m_translucentSurface)
        ImGui::SliderFloat("Opacity", &m_surfaceOpacity, 0.05f, 0.95f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::EndDisabled();
    if (ImGui::SliderInt("Mesh resolution", &m_meshResolution, 1, kMaxMeshResolution, "%d",
                         ImGuiSliderFlags_AlwaysClamp))
    {
        m_meshDirty = true;
    }
    ImGui::SetItemTooltip("Grid cells along each of u and v (coarser while a point is dragged)");
    ImGui::Checkbox("Show isocurves", &m_showIsocurves);
    if (ImGui::SliderInt("Isocurves", &m_isocurveCount, 2, 40, "%d", ImGuiSliderFlags_AlwaysClamp))
        m_meshDirty = true;
    if (SampleCountSlider(m_sampleCount))
        m_meshDirty = true;
    ImGui::Checkbox("Show control net", &m_showNet);
    ImGui::Checkbox("Label all points", &m_labelAllPoints);
    ImGui::SetItemTooltip("Otherwise only the point under the mouse and the selected one are labeled");
    bool orthographic = m_view.Camera().IsOrthographic();
    if (ImGui::Checkbox("Orthographic", &orthographic))
        m_view.Camera().SetOrthographic(orthographic);
}

bool SurfaceView::DrawFooter() const
{
    ImGui::Spacing();
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("%s", Graphics::Viewport3D::kDragHint);
    ImGui::PopTextWrapPos();
    return Graphics::FitViewFooter(Graphics::Viewport3D::kNavigationHint);
}

bool SurfaceView::Begin(const char* id, std::span<glm::dvec3> points)
{
    m_view.Begin(id);
    if (m_drag.active >= static_cast<int>(points.size()) || m_selected >= static_cast<int>(points.size()))
        ClearSelection();

    const bool moved = m_view.DragPoints(points, m_drag);
    if (Dragging())
        m_selected = m_drag.active;
    if (moved)
        m_meshDirty = true;
    return moved;
}

void SurfaceView::Draw(const Surface& surface, std::span<const glm::dvec3> points, std::size_t columns,
                       std::span<const double> weights)
{
    assert(columns > 0 && points.size() % columns == 0);
    assert(weights.empty() || weights.size() == points.size());

    UpdateMesh(surface);
    if (m_showSurface)
        m_view.DrawMesh(m_mesh, kSurfaceFrontColor, kSurfaceBackColor, m_translucentSurface ? m_surfaceOpacity : 1.0f);
    if (m_showIsocurves)
    {
        for (const std::vector<glm::dvec3>& curve : m_isocurves)
            m_view.Polyline(curve, kIsocurveColor, 1.5f);
    }

    // The isocurve S(u, v0), and its part from 0 to u0 by an evenly spaced share of the samples.
    const auto samples = static_cast<std::size_t>(m_sampleCount);
    const auto isocurve = [&surface, this](double uEnd, std::size_t count) {
        return Graphics::SampleCurve([&surface, this](double u) { return surface(u, m_v0); }, 0.0, uEnd, count);
    };
    m_view.Polyline(isocurve(1.0, samples), kIsoColor, 2.5f);
    if (m_u0 > 0.0)
    {
        const auto progressSamples =
            std::max<std::size_t>(2, static_cast<std::size_t>(std::lround(m_u0 * static_cast<double>(samples))));
        m_view.Polyline(isocurve(m_u0, progressSamples), kProgressColor, 4.0f);
    }

    if (m_showNet)
        DrawControlNet(points, columns);
    DrawControlPoints(points, columns, weights);
    DrawDragCues(points);

    DrawResultPoint(m_view, surface(m_u0, m_v0), "S(u0, v0)");
}

void SurfaceView::UpdateMesh(const Surface& surface)
{
    const int resolution = Dragging() ? std::min(m_meshResolution, kDragMeshResolution) : m_meshResolution;
    if (!m_meshDirty && resolution == m_builtResolution)
        return;

    const auto cells = static_cast<std::size_t>(resolution);
    m_mesh.Upload(Graphics::SampleSurfaceMesh(surface, 0.0, 1.0, 0.0, 1.0, cells, cells));

    // Curves of constant u (sampled along v), then of constant v (sampled along u).
    const auto count = static_cast<std::size_t>(m_isocurveCount);
    const auto samples = static_cast<std::size_t>(Dragging() ? std::min(m_sampleCount, kDragSampleCount) : m_sampleCount);
    m_isocurves.clear();
    for (std::size_t k = 0; k < count; ++k)
    {
        const double u = Graphics::ParameterAt(0.0, 1.0, k, count);
        m_isocurves.push_back(Graphics::SampleCurve([&surface, u](double v) { return surface(u, v); }, 0.0, 1.0, samples));
    }
    for (std::size_t k = 0; k < count; ++k)
    {
        const double v = Graphics::ParameterAt(0.0, 1.0, k, count);
        m_isocurves.push_back(Graphics::SampleCurve([&surface, v](double u) { return surface(u, v); }, 0.0, 1.0, samples));
    }

    m_meshDirty = false;
    m_builtResolution = resolution;
}

void SurfaceView::DrawControlNet(std::span<const glm::dvec3> points, std::size_t columns)
{
    const std::size_t rows = points.size() / columns;
    // Each row P[i][0..m] is contiguous; each column P[0..n][j] is strided.
    for (std::size_t i = 0; i < rows; ++i)
        m_view.Polyline(points.subspan(i * columns, columns), kNetColor, 1.0f);
    for (std::size_t j = 0; j < columns; ++j)
    {
        for (std::size_t i = 1; i < rows; ++i)
            m_view.Line(points[(i - 1) * columns + j], points[i * columns + j], kNetColor, 1.0f);
    }
}

void SurfaceView::DrawControlPoints(std::span<const glm::dvec3> points, std::size_t columns,
                                    std::span<const double> weights)
{
    // A point hovered in the domain inset is highlighted here too.
    const int underMouse = PointUnderMouse() >= 0 ? PointUnderMouse() : m_domainHovered;
    for (std::size_t k = 0; k < points.size(); ++k)
    {
        const auto index = static_cast<int>(k);
        const bool dragged = index == m_drag.active;
        ImU32 color = kControlPointColor;
        if (dragged)
            color = kDraggedColor;
        else if (index == underMouse)
            color = kHoveredColor;
        else if (index == m_selected)
            color = kSelectedColor;

        const bool emphasized = index == underMouse || index == m_selected;
        const double* weight = weights.empty() ? nullptr : &weights[k];
        const float radius = PointRadius(emphasized) * (weight != nullptr ? WeightScale(*weight) : 1.0f);
        m_view.Point(points[k], color, radius);
        if (m_labelAllPoints || emphasized)
            m_view.Label(points[k], ControlPointLabel(k / columns, k % columns, weight).c_str(), color);
    }
}

void SurfaceView::DrawDragCues(std::span<const glm::dvec3> points)
{
    if (!Dragging())
        return;

    const glm::dvec3 p = points[static_cast<std::size_t>(m_drag.active)];
    const glm::dvec3 start = m_drag.start;
    m_view.Line(start, p, kGhostColor, 1.0f);
    m_view.Point(start, kGhostColor, kPointRadius);

    // Down (or up) to the z = 0 grid, with the point's shadow there.
    const glm::dvec3 shadow{p.x, p.y, 0.0};
    m_view.Line(p, shadow, kDropColor, 1.0f);
    m_view.Point(shadow, kDropColor, 3.0f);

    // The line or plane the point is held to.
    const double half = kGuideShare * m_extent;
    switch (m_drag.drag.Constraint())
    {
    case Graphics::DragConstraint::Vertical:
        m_view.Line(p - glm::dvec3{0.0, 0.0, half}, p + glm::dvec3{0.0, 0.0, half}, Graphics::Theme::kZAxisColor, 1.5f);
        break;
    case Graphics::DragConstraint::Horizontal:
        m_view.Line(p - glm::dvec3{half, 0.0, 0.0}, p + glm::dvec3{half, 0.0, 0.0}, Graphics::Theme::kXAxisColor, 1.5f);
        m_view.Line(p - glm::dvec3{0.0, half, 0.0}, p + glm::dvec3{0.0, half, 0.0}, Graphics::Theme::kYAxisColor, 1.5f);
        break;
    case Graphics::DragConstraint::ViewPlane:
        break;
    }

    // The coordinates, under the point's own label.
    if (const std::optional<ImVec2> s = m_view.ToScreen(p))
    {
        const std::string text = std::format("({:.3f}, {:.3f}, {:.3f})", p.x, p.y, p.z);
        const ImVec2 below(s->x, s->y + ImGui::GetFontSize() + 4.0f);
        Graphics::DrawLabel(ImGui::GetWindowDrawList(), below, text.c_str(), kDraggedColor,
                            Graphics::Theme::kLabelShadowColor);
    }
}

} // namespace NURBS::Viewer
