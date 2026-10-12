#pragma once

#include <Graphics/Scene.hpp>
#include <Graphics/Viewport2D.hpp>

#include <glm/vec2.hpp>
#include <imgui.h>

#include <cstddef>
#include <vector>

namespace NURBS::Viewer
{

// Illustrates Algorithms A2.1 (FindSpan), A2.2 (BasisFuns), A2.3
// (DersBasisFuns), A2.4 (OneBasisFun) and A2.5 (DersOneBasisFun): the graphs
// of the degree p B-spline basis N_{k,p}(u), or of a chosen derivative of it,
// over an editable knot vector, the knot span u0 lies in, the p+1 functions
// nonzero on it, the triangle of lower-degree functions the Cox-de Boor
// recurrence builds them from, and the table of their derivatives at u0. A
// single function can be focused, showing its support, the inverted triangle
// A2.4 builds it from, and its derivatives from A2.5.
class BSplineBasisScene final : public Graphics::Scene
{
public:
    BSplineBasisScene();

    [[nodiscard]] const char* Name() const override { return "B-spline basis (A2.1-A2.5)"; }
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
    void DrawDersTable(std::size_t span) const;
    // The focused function at u0 from OneBasisFun, and its inverted triangle.
    void DrawOneFunction(std::size_t span) const;
    // The derivatives of the focused function at u0 from DersOneBasisFun.
    void DrawOneFunctionDers(std::size_t span) const;
    // Draws the knots on a rail below the graphs and lets them be dragged.
    void DrawKnotRail();

    // The number of basis functions less one: n = m - p - 1.
    [[nodiscard]] std::size_t LastIndex() const;
    // [u_p, u_{n+1}], where the basis sums to one.
    [[nodiscard]] glm::dvec2 Domain() const;
    // The height of the knot rail, just below the graphs.
    [[nodiscard]] double RailY() const;
    // True when the partition-of-unity bar is drawn, which is only for the
    // functions themselves.
    [[nodiscard]] bool ShowsPartition() const;
    // True when one function is focused.
    [[nodiscard]] bool Focuses() const;
    // The graph of the m_derivOrder-th derivative of every N_{k,p}:
    // basis[k][s] = (u_s, N^(order)_{k,p}(u_s)). At an interior knot it has
    // both the left and the right limit, so jumps are drawn upright.
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

    // The derivative order k graphed, from 0 (the functions) to p.
    int m_derivOrder = 0;

    bool m_showPartition = true;
    bool m_showTriangle = true;
    bool m_showTangents = false;

    // The index i of the focused function N_{i,p}, or -1 for none.
    int m_focus = -1;

    // The index of the knot being dragged on the rail, or -1.
    int m_draggedKnot = -1;

    Graphics::Viewport2D m_view;
};

} // namespace NURBS::Viewer
