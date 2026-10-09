#pragma once

#include "Scene.hpp"
#include "Viewport2D.hpp"

#include <NURBS/bezier.hpp>

#include <glm/vec2.hpp>

#include <cstddef>
#include <vector>

namespace NURBS::Viewer
{

// Illustrates NURBS::BezierCurve and Algorithms A1.4 (PointOnBezierCurve) and
// A1.5 (deCasteljau1): a 2D Bezier curve C(u) = sum B_{k,n}(u) * P[k],
// k = 0..n, whose control points can be dragged, and the point C(u0) on it.
class BezierScene final : public Scene
{
public:
    BezierScene();

    [[nodiscard]] const char* Name() const override { return "Bezier (A1.4, A1.5)"; }
    void DrawUI() override;
    void DrawViewport() override;

private:
    void LoadPreset(std::size_t preset);
    void SetDegree(int degree);
    void FitView();
    void DrawValuesTable() const;

    [[nodiscard]] std::vector<glm::dvec2> SampledCurve() const;

    // Its control points are dragged and edited in place; only a degree change
    // or a preset builds a new curve.
    NURBS::BezierCurve<glm::dvec2> m_curve;
    double m_u0 = 0.5;
    int m_sampleCount = 200;
    std::size_t m_preset = 0;

    // When on, the control polygon P[0], P[1], ..., P[n] is drawn under the curve.
    bool m_showPolygon = true;

    Viewport2D m_view;
    // The control point being dragged in m_view; -1 when none is.
    int m_draggedPoint = -1;
};

} // namespace NURBS::Viewer
