#pragma once

#include <Graphics/Scene.hpp>
#include <Graphics/Viewport2D.hpp>

#include <glm/vec2.hpp>
#include <imgui.h>

#include <cstddef>
#include <vector>

namespace NURBS::Viewer
{

// Illustrates Algorithms A2.1 (FindSpan) and A2.2 (BasisFuns): the graphs of
// the degree p B-spline basis N_{k,p}(u) over an editable knot vector, the
// knot span u0 lies in, the p+1 functions nonzero on it, and the triangle of
// lower-degree functions the Cox-de Boor recurrence builds them from.
class BSplineBasisScene final : public Graphics::Scene
{
public:
    BSplineBasisScene();

    [[nodiscard]] const char* Name() const override { return "B-spline basis (A2.1, A2.2)"; }
    void DrawUI() override;
    void DrawViewport() override;
    void DrawViewOptions() override;

private:
    void LoadPreset(std::size_t preset);
    void FitView();
    // Replaces the knots with knots if they form a valid knot vector for the
    // degree, keeping u0 in the new domain. Returns true if they were taken.
    bool SetKnots(std::vector<double> knots);

    void DrawKnotEditor();
    void DrawValuesTable(std::size_t span, const std::vector<double>& values) const;
    void DrawTriangle(std::size_t span) const;
    // Draws the knots on a rail below the graphs and lets them be dragged.
    void DrawKnotRail();

    // The number of basis functions less one: n = m - p - 1.
    [[nodiscard]] std::size_t LastIndex() const;
    // [u_p, u_{n+1}], where the basis sums to one.
    [[nodiscard]] glm::dvec2 Domain() const;
    // The graph of every N_{k,p}: basis[k][s] = (u_s, N_{k,p}(u_s)).
    [[nodiscard]] std::vector<std::vector<glm::dvec2>> SampledBasis() const;
    [[nodiscard]] ImU32 ColorOf(std::size_t k, float alpha = 1.0f) const;

    int m_degree = 3;
    std::vector<double> m_knots;
    std::size_t m_preset = 0;

    double m_u0 = 0.5;
    // u0 wraps from the end of the domain back to the start, taking
    // m_sweepSeconds for one pass.
    bool m_animateU = false;
    float m_sweepSeconds = 4.0f;
    int m_sampleCount = 200;

    bool m_showPartition = true;
    bool m_showTriangle = true;

    // The index of the knot being dragged on the rail, or -1.
    int m_draggedKnot = -1;

    Graphics::Viewport2D m_view;
};

} // namespace NURBS::Viewer
