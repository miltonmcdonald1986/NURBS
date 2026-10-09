#pragma once

#include "Scene.hpp"
#include "Viewport2D.hpp"

#include <NURBS/bezier.hpp>

#include <glm/vec2.hpp>

#include <cstddef>
#include <vector>

namespace NURBS::Viewer
{

// Illustrates NURBS::RationalBezierCurve (The NURBS Book, Eq. 4.1): a 2D
// rational Bezier curve C(u) = sum B_{k,n}(u) w_k P_k / sum B_{k,n}(u) w_k,
// k = 0..n, whose control points can be dragged and whose weights can be
// edited, and the point C(u0) on it.
class RationalBezierScene final : public Scene
{
public:
    RationalBezierScene();

    [[nodiscard]] const char* Name() const override { return "Rational Bezier (Eq. 4.1)"; }
    void DrawUI() override;
    void DrawViewport() override;

private:
    void LoadPreset(std::size_t preset);
    void SetDegree(int degree);
    void FitView();
    void DrawValuesTable() const;
    void DrawConicType() const;

    [[nodiscard]] std::vector<glm::dvec2> SampledCurve() const;
    // The BezierCurve on the same control points, i.e. this curve with every weight 1.
    [[nodiscard]] std::vector<glm::dvec2> SampledPolynomialCurve() const;

    // Its control points and weights are edited in place; only a degree change
    // or a preset builds a new curve.
    NURBS::RationalBezierCurve<glm::dvec2> m_curve;
    double m_u0 = 0.5;
    int m_sampleCount = 200;
    std::size_t m_preset = 0;

    // When on, the control polygon P[0], P[1], ..., P[n] is drawn under the curve.
    bool m_showPolygon = true;
    // When on, the control point labels show their weights.
    bool m_showWeights = true;
    // When on, the polynomial curve (every weight 1) is drawn for comparison.
    bool m_showPolynomial = false;

    Viewport2D m_view;
    // The control point being dragged in m_view; -1 when none is.
    int m_draggedPoint = -1;
};

} // namespace NURBS::Viewer
