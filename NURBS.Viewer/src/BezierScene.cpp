#include "BezierScene.hpp"

#include "CurveSampling.hpp"
#include "ImGuiHelpers.hpp"

#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <numbers>
#include <span>

namespace NURBS::Viewer
{

namespace
{

constexpr int kMaxDegree = 10;
constexpr std::size_t kDefaultPreset = 1;

constexpr ImU32 kCurveColor = IM_COL32(90, 170, 255, 255);
constexpr ImU32 kPolygonColor = IM_COL32(120, 120, 140, 200);
constexpr ImU32 kControlPointColor = IM_COL32(240, 180, 60, 255);
constexpr ImU32 kDraggedColor = IM_COL32(255, 255, 255, 255);
constexpr ImU32 kResultColor = IM_COL32(255, 90, 90, 255);

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

// How sharply each appended control point turns the control polygon, and how
// much shorter its new leg is than the previous one.
constexpr double kTurnRadians = std::numbers::pi / 4.0;
constexpr double kShrink = 0.75;
// The new leg when there is no previous one to follow.
constexpr glm::dvec2 kFirstLeg{0.5, 0.0};

// The control point appended when the degree slider raises the degree by one.
// Each new leg is the previous one turned toward the other points and shrunk,
// so repeated calls wind the polygon into an inward spiral: the curve keeps
// bending back, and since the leg lengths form a geometric series the points
// stay within 1 / (1 - kShrink) legs of where they started.
// Precondition: points is non-empty.
[[nodiscard]] glm::dvec2 NextControlPoint(std::span<const glm::dvec2> points)
{
    assert(!points.empty());

    const glm::dvec2 last = points.back();
    if (points.size() == 1)
        return last + kFirstLeg;
    const glm::dvec2 leg = last - points[points.size() - 2];
    if (glm::dot(leg, leg) == 0.0)
        return last + kFirstLeg;

    glm::dvec2 centroid{0.0, 0.0};
    for (const glm::dvec2& p : points)
        centroid += p;
    centroid /= static_cast<double>(points.size());

    // Turn counterclockwise if the centroid is left of the leg, else clockwise.
    const glm::dvec2 toCentroid = centroid - last;
    const double cross = leg.x * toCentroid.y - leg.y * toCentroid.x;
    const double angle = cross >= 0.0 ? kTurnRadians : -kTurnRadians;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const glm::dvec2 turned{c * leg.x - s * leg.y, s * leg.x + c * leg.y};
    return last + kShrink * turned;
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
    m_curve = NURBS::BezierCurve<glm::dvec2>(Presets()[preset].controlPoints);
    m_draggedPoint = -1;
    FitView();
}

void BezierScene::SetDegree(int degree)
{
    // A BezierCurve's degree is fixed, so build a new one from the points kept.
    // TODO: When the library has Bezier degree elevation (The NURBS Book, Section 5.5),
    // raise the degree with it so the curve keeps its shape, and remove
    // NextControlPoint. Lowering the degree can keep truncating, since degree
    // reduction is only approximate.
    const auto count = static_cast<std::size_t>(degree) + 1;
    const std::span<const glm::dvec2> kept = m_curve.ControlPoints().first(std::min(count, m_curve.ControlPoints().size()));
    std::vector<glm::dvec2> points(kept.begin(), kept.end());
    while (points.size() < count)
    {
        const glm::dvec2 next = NextControlPoint(points);
        points.push_back(next);
    }
    m_curve = NURBS::BezierCurve<glm::dvec2>(points);
    m_draggedPoint = -1;
}

void BezierScene::FitView()
{
    // By the convex hull property the curve lies within the control points' box.
    const Bounds2D bounds = *BoundsOf(m_curve.ControlPoints());
    m_view.FitTo(bounds.min, bounds.max);
}

std::vector<glm::dvec2> BezierScene::SampledCurve() const
{
    return SampleCurve([this](double u) { return m_curve.Evaluate(u); }, 0.0, 1.0,
                       static_cast<std::size_t>(m_sampleCount));
}

void BezierScene::DrawUI()
{
    if (PresetCombo(Presets(), m_preset, [](const Preset& preset) { return preset.name; }))
        LoadPreset(m_preset);

    int degree = static_cast<int>(m_curve.Degree());
    if (ImGui::SliderInt("Degree n", &degree, 0, kMaxDegree, "%d", ImGuiSliderFlags_AlwaysClamp))
        SetDegree(degree);

    ImGui::SeparatorText("Control points P[i]");
    const std::span<glm::dvec2> points = m_curve.ControlPoints();
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        ImGui::DragScalarN(IndexedLabel("P", i).c_str(), ImGuiDataType_Double, glm::value_ptr(points[i]), 2, 0.01f, nullptr,
                           nullptr, "%.3f");
    }
    ImGui::TextDisabled("Left drag a point in the view to move it");

    ImGui::SeparatorText("Parameter");
    UnitSlider("u0", m_u0);
    SampleCountSlider(m_sampleCount);

    ImGui::SeparatorText("Display");
    ImGui::Checkbox("Show control polygon", &m_showPolygon);

    ImGui::SeparatorText("Point at u0");
    DrawValuesTable();

    ImGui::Spacing();
    ImGui::TextWrapped("C(u) = sum B_{k,n}(u) P_k,  k = 0..n");
    ImGui::TextWrapped("A1.4 sums the Bernstein basis times the control points. A1.5 (de Casteljau, used by "
                       "BezierCurve::Evaluate) repeatedly interpolates P_i = (1-u) P_i + u P_{i+1}.");

    if (FitViewFooter())
        FitView();
}

void BezierScene::DrawValuesTable() const
{
    if (!BeginTableWithHeaders("values", {"Algorithm", "C(u0)"}))
        return;

    const glm::dvec2 deCasteljau = m_curve.Evaluate(m_u0);
    const glm::dvec2 bernstein = NURBS::PointOnBezierCurve(m_curve.ControlPoints(), m_u0);

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("A1.5 Evaluate");
    ImGui::TableNextColumn();
    TextPoint(deCasteljau);

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("A1.4 PointOnBezierCurve");
    ImGui::TableNextColumn();
    TextPoint(bernstein);

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("|diff|");
    ImGui::TableNextColumn();
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
        m_view.Polyline(points, kPolygonColor, 1.0f);
    m_view.Polyline(SampledCurve(), kCurveColor, 2.5f);

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const bool dragged = static_cast<int>(i) == m_draggedPoint;
        m_view.LabeledPoint(points[i], IndexedLabel("P", i).c_str(), dragged ? kDraggedColor : kControlPointColor,
                            dragged ? 6.0f : 4.0f);
    }

    m_view.LabeledPoint(m_curve.Evaluate(m_u0), "C(u0)", kResultColor, 6.0f);

    m_view.End();
}

} // namespace NURBS::Viewer
