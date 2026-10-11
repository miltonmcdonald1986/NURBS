#include "BezierScene.hpp"

#include "ControlPolygon.hpp"
#include "Palette.hpp"
#include "SceneWidgets.hpp"

#include <Graphics/Animation.hpp>
#include <Graphics/Sampling.hpp>

#include <glm/geometric.hpp>
#include <imgui.h>

#include <array>
#include <span>

namespace NURBS::Viewer
{

namespace
{

// The domain of u0.
constexpr double kUMin = 0.0;
constexpr double kUMax = 1.0;

constexpr int kMaxDegree = 10;
constexpr std::size_t kDefaultPreset = 1;


struct Preset
{
    const char* name;
    std::vector<glm::dvec2> controlPoints;
};

const std::array<Preset, 4>& Presets()
{
    static const std::array<Preset, 4> presets{{
        {"Quadratic", {{0.0, 0.0}, {0.5, 1.0}, {1.0, 0.0}}},
        // The same curve as the Horner scene's "Cubic arch" preset.
        {"Cubic arch", {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}}},
        {"S-curve", {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}}},
        // Crossed middle control points make the curve loop.
        {"Loop", {{0.0, 0.0}, {1.5, 1.0}, {-0.5, 1.0}, {1.0, 0.0}}},
    }};
    return presets;
}

} // namespace

BezierScene::BezierScene()
    : m_curve(Presets()[kDefaultPreset].controlPoints)
    , m_preset(kDefaultPreset)
{
    FitView();
}

void BezierScene::LoadPreset(std::size_t preset)
{
    m_preset = preset;
    m_animateU = false;
    m_curve = NURBS::BezierCurve<glm::dvec2>(Presets()[preset].controlPoints);
    m_draggedPoint = -1;
    FitView();
}

void BezierScene::SetDegree(int degree)
{
    // A BezierCurve's degree is fixed, so build a new one from the points kept.
    const std::vector<glm::dvec2> points =
        ResizeControlPolygon(m_curve.ControlPoints(), static_cast<std::size_t>(degree) + 1);
    m_curve = NURBS::BezierCurve<glm::dvec2>(points);
    m_draggedPoint = -1;
}

void BezierScene::FitView()
{
    FitToControlPoints(m_view, m_curve.ControlPoints());
}

std::vector<glm::dvec2> BezierScene::SampledCurve() const
{
    return Graphics::SampleCurve([this](double u) { return m_curve.Evaluate(u); }, 0.0, 1.0,
                       static_cast<std::size_t>(m_sampleCount));
}

void BezierScene::DrawUI()
{
    if (PresetCombo(Presets(), m_preset))
        LoadPreset(m_preset);

    int degree = static_cast<int>(m_curve.Degree());
    if (DegreeSlider("Degree n", degree, kMaxDegree))
        SetDegree(degree);

    ImGui::SeparatorText("Control points P[i]");
    const std::span<glm::dvec2> points = m_curve.ControlPoints();
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        Graphics::DragPoint2(Graphics::IndexedLabel("P", i).c_str(), points[i]);
    }
    ImGui::TextDisabled("Left drag a point in the view to move it");

    ImGui::SeparatorText("Parameter");
    if (m_animateU)
        Graphics::AdvanceWrapping(m_u0, kUMin, kUMax, SweepStep(m_sweepSeconds));
    Graphics::PlayPauseSlider("u0", "###animateU", "Animate u0: C(u0) moves along the curve",
                              m_u0, kUMin, kUMax, m_animateU);
    SweepTimeSlider(m_sweepSeconds);
    SampleCountSlider(m_sampleCount);

    ImGui::SeparatorText("Display");
    ImGui::Checkbox("Show control polygon", &m_showPolygon);

    ImGui::SeparatorText("Point at u0");
    DrawValuesTable();

    ImGui::Spacing();
    ImGui::TextWrapped("C(u) = sum B_{k,n}(u) P_k,  k = 0..n");
    ImGui::TextWrapped("A1.4 sums the Bernstein basis times the control points. A1.5 (de Casteljau, used by "
                       "BezierCurve::Evaluate) repeatedly interpolates P_i = (1-u) P_i + u P_{i+1}.");

    if (Graphics::FitViewFooter(Graphics::Viewport2D::kNavigationHint))
        FitView();
}

void BezierScene::DrawValuesTable() const
{
    if (!Graphics::BeginTableWithHeaders("values", {"Algorithm", "C(u0)"}))
        return;

    const glm::dvec2 deCasteljau = m_curve.Evaluate(m_u0);
    const glm::dvec2 bernstein = NURBS::PointOnBezierCurve(m_curve.ControlPoints(), m_u0);

    Graphics::TableRowPoint("A1.5 Evaluate", deCasteljau);
    Graphics::TableRowPoint("A1.4 PointOnBezierCurve", bernstein);
    Graphics::TableRowLabel("|diff|");
    ImGui::Text("%.1e", glm::distance(deCasteljau, bernstein));
    ImGui::EndTable();
}

void BezierScene::DrawViewport()
{
    m_view.Begin("##bezier");

    // Edit first so this frame draws the moved point.
    const std::span<glm::dvec2> points = m_curve.ControlPoints();
    m_view.DragPoints(points, m_draggedPoint);

    if (m_showPolygon)
        DrawControlPolygon(m_view, points);
    m_view.Polyline(SampledCurve(), kCurveColor, 2.5f);
    DrawControlPoints(m_view, points, m_draggedPoint);

    DrawResultPoint(m_view, m_curve.Evaluate(m_u0), "C(u0)");

    m_view.End();
}

} // namespace NURBS::Viewer
