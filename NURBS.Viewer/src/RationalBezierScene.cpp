#include "RationalBezierScene.hpp"

#include "ControlPolygon.hpp"
#include "CurveSampling.hpp"
#include "ImGuiHelpers.hpp"

#include <NURBS/bernstein.hpp>

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
constexpr std::size_t kDefaultPreset = 0;

// The weight sliders' range. Weights must stay positive, so the slider is
// clamped even when a value is typed in.
constexpr double kMinWeight = 0.01;
constexpr double kMaxWeight = 100.0;

constexpr ImU32 kCurveColor = IM_COL32(90, 170, 255, 255);
constexpr ImU32 kPolynomialColor = IM_COL32(110, 220, 120, 160);
constexpr ImU32 kResultColor = IM_COL32(255, 90, 90, 255);

struct Preset
{
    const char* name;
    std::vector<glm::dvec2> controlPoints;
    std::vector<double> weights;
};

const std::array<Preset, 6>& Presets()
{
    // The first four share a control polygon and differ only in the middle
    // weight, which picks the type of conic (see DrawConicType).
    static const std::array<Preset, 6> presets{{
        {"Quarter circle", {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, {1.0, std::numbers::sqrt2 / 2.0, 1.0}},
        {"Ellipse arc", {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, {1.0, 0.5, 1.0}},
        {"Parabola", {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, {1.0, 1.0, 1.0}},
        {"Hyperbola", {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, {1.0, 2.0, 1.0}},
        // A semicircle of radius 1 about the origin as one cubic.
        {"Semicircle", {{1.0, 0.0}, {1.0, 2.0}, {-1.0, 2.0}, {-1.0, 0.0}}, {1.0, 1.0 / 3.0, 1.0 / 3.0, 1.0}},
        // The Bezier scene's "Cubic arch", pulled toward P1.
        {"Weighted cubic arch", {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}}, {1.0, 3.0, 1.0, 1.0}},
    }};
    return presets;
}

// C(u) evaluated straight from Eq. 4.1, with the basis from AllBernstein (A1.3):
// sum B_{k,n}(u) w_k P_k / sum B_{k,n}(u) w_k.
[[nodiscard]] glm::dvec2 PointFromBasis(std::span<const glm::dvec2> points, std::span<const double> weights, double u)
{
    assert(!points.empty());
    assert(weights.size() == points.size());

    const std::vector<double> B = NURBS::AllBernstein(points.size() - 1, u);
    glm::dvec2 numerator{0.0, 0.0};
    double denominator = 0.0;
    for (std::size_t k = 0; k < points.size(); ++k)
    {
        numerator += B[k] * weights[k] * points[k];
        denominator += B[k] * weights[k];
    }
    return numerator / denominator;
}

} // namespace

RationalBezierScene::RationalBezierScene()
    : m_curve(Presets()[kDefaultPreset].controlPoints, Presets()[kDefaultPreset].weights)
    , m_preset(kDefaultPreset)
{
    FitView();
}

void RationalBezierScene::LoadPreset(std::size_t preset)
{
    m_preset = preset;
    m_curve = NURBS::RationalBezierCurve<glm::dvec2>(Presets()[preset].controlPoints, Presets()[preset].weights);
    m_draggedPoint = -1;
    FitView();
}

void RationalBezierScene::SetDegree(int degree)
{
    // A RationalBezierCurve's degree is fixed, so build a new one from the
    // points and weights kept. Appended points get weight 1.
    const auto count = static_cast<std::size_t>(degree) + 1;
    const std::vector<glm::dvec2> points = ResizeControlPolygon(m_curve.ControlPoints(), count);
    const std::span<const double> kept = m_curve.Weights().first(std::min(count, m_curve.Weights().size()));
    std::vector<double> weights(kept.begin(), kept.end());
    weights.resize(count, 1.0);
    m_curve = NURBS::RationalBezierCurve<glm::dvec2>(points, weights);
    m_draggedPoint = -1;
}

void RationalBezierScene::FitView()
{
    // With positive weights the curve still lies within the control points' convex hull.
    const Bounds2D bounds = *BoundsOf(m_curve.ControlPoints());
    m_view.FitTo(bounds.min, bounds.max);
}

std::vector<glm::dvec2> RationalBezierScene::SampledCurve() const
{
    return SampleCurve([this](double u) { return m_curve.Evaluate(u); }, 0.0, 1.0,
                       static_cast<std::size_t>(m_sampleCount));
}

std::vector<glm::dvec2> RationalBezierScene::SampledPolynomialCurve() const
{
    return SampleCurve([this](double u) { return NURBS::deCasteljau1(m_curve.ControlPoints(), u); }, 0.0, 1.0,
                       static_cast<std::size_t>(m_sampleCount));
}

void RationalBezierScene::DrawUI()
{
    if (PresetCombo(Presets(), m_preset, [](const Preset& preset) { return preset.name; }))
        LoadPreset(m_preset);

    int degree = static_cast<int>(m_curve.Degree());
    if (ImGui::SliderInt("Degree n", &degree, 0, kMaxDegree, "%d", ImGuiSliderFlags_AlwaysClamp))
        SetDegree(degree);

    ImGui::SeparatorText("Control points P[i] and weights w[i]");
    const std::span<glm::dvec2> points = m_curve.ControlPoints();
    const std::span<double> weights = m_curve.Weights();
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        ImGui::DragScalarN(IndexedLabel("P", i).c_str(), ImGuiDataType_Double, glm::value_ptr(points[i]), 2, 0.01f, nullptr,
                           nullptr, "%.3f");
        ImGui::SliderScalar(IndexedLabel("w", i).c_str(), ImGuiDataType_Double, &weights[i], &kMinWeight, &kMaxWeight,
                            "%.3f", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
    }
    if (ImGui::Button("Reset weights"))
        std::ranges::fill(weights, 1.0);
    ImGui::SetItemTooltip("Set every weight to 1, which gives the polynomial Bezier curve");
    ImGui::TextDisabled("Left drag a point in the view to move it");

    ImGui::SeparatorText("Parameter");
    UnitSlider("u0", m_u0);
    SampleCountSlider(m_sampleCount);

    ImGui::SeparatorText("Display");
    ImGui::Checkbox("Show control polygon", &m_showPolygon);
    ImGui::Checkbox("Show weights", &m_showWeights);
    ImGui::Checkbox("Show polynomial curve (all w = 1)", &m_showPolynomial);

    ImGui::SeparatorText("Point at u0");
    DrawValuesTable();
    DrawConicType();

    ImGui::Spacing();
    ImGui::TextWrapped("C(u) = sum B_{k,n}(u) w_k P_k / sum B_{k,n}(u) w_k,  k = 0..n");
    ImGui::TextWrapped("Raising w_k pulls the curve toward P_k; lowering it pushes the curve away. Scaling every "
                       "weight by the same factor leaves the curve unchanged. RationalBezierCurve::Evaluate runs "
                       "de Casteljau (A1.5) on the homogeneous points (w_k P_k, w_k) and divides by the weight.");

    if (FitViewFooter())
        FitView();
}

void RationalBezierScene::DrawValuesTable() const
{
    if (!BeginTableWithHeaders("values", {"Algorithm", "C(u0)"}))
        return;

    const glm::dvec2 homogeneous = m_curve.Evaluate(m_u0);
    const glm::dvec2 basis = PointFromBasis(m_curve.ControlPoints(), m_curve.Weights(), m_u0);

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Evaluate (homogeneous A1.5)");
    ImGui::TableNextColumn();
    TextPoint(homogeneous);

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Eq. 4.1 with AllBernstein");
    ImGui::TableNextColumn();
    TextPoint(basis);

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("|diff|");
    ImGui::TableNextColumn();
    ImGui::Text("%.1e", glm::distance(homogeneous, basis));

    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("w(u0) = sum B_{k,n} w_k");
    ImGui::TableNextColumn();
    ImGui::Text("%.3f", NURBS::deCasteljau1(m_curve.Weights(), m_u0));
    ImGui::EndTable();
}

void RationalBezierScene::DrawConicType() const
{
    // Every quadratic rational Bezier curve is a conic arc, and its type is
    // set by the conic shape factor k = w0 w2 / w1^2 (The NURBS Book, Section 7.3):
    // an ellipse when k > 1, a parabola when k = 1, and a hyperbola when k < 1.
    if (m_curve.Degree() != 2)
        return;

    const std::span<const double> w = m_curve.Weights();
    const double k = w[0] * w[2] / (w[1] * w[1]);
    constexpr double kTolerance = 1e-9;
    const char* type = "parabola";
    if (k > 1.0 + kTolerance)
        type = "ellipse";
    else if (k < 1.0 - kTolerance)
        type = "hyperbola";

    ImGui::Spacing();
    ImGui::Text("Conic arc: %s", type);
    ImGui::SetItemTooltip("k = w0 w2 / w1^2 = %.3f\nk > 1: ellipse, k = 1: parabola, k < 1: hyperbola", k);
}

void RationalBezierScene::DrawViewport()
{
    m_view.Begin("##rationalBezier");

    // Edit first so this frame draws the moved point.
    const std::span<glm::dvec2> points = m_curve.ControlPoints();
    m_view.DragPoints(points, m_draggedPoint);

    if (m_showPolygon)
        DrawControlPolygon(m_view, points);
    if (m_showPolynomial)
        m_view.Polyline(SampledPolynomialCurve(), kPolynomialColor, 1.5f);
    m_view.Polyline(SampledCurve(), kCurveColor, 2.5f);
    const std::span<const double> weights = m_curve.Weights();
    DrawControlPoints(m_view, points, m_draggedPoint, m_showWeights ? weights : std::span<const double>{});

    m_view.LabeledPoint(m_curve.Evaluate(m_u0), "C(u0)", kResultColor, 6.0f);

    m_view.End();
}

} // namespace NURBS::Viewer
