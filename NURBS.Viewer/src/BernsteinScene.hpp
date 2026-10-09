#pragma once

#include "Scene.hpp"
#include "Viewport2D.hpp"

#include <glm/vec2.hpp>
#include <imgui.h>

#include <cstddef>
#include <vector>

namespace NURBS::Viewer
{

// Illustrates Algorithms A1.2 (Bernstein) and A1.3 (AllBernstein): the graphs
// of the degree n Bernstein basis B_{i,n}(u), i = 0..n, over u in [0, 1], and
// their values at u0, which are non-negative and sum to one.
class BernsteinScene final : public Scene
{
public:
    BernsteinScene();

    [[nodiscard]] const char* Name() const override { return "Bernstein (A1.2, A1.3)"; }
    void DrawUI() override;
    void DrawViewport() override;

private:
    void FitView();
    void DrawValuesTable(const std::vector<double>& values) const;

    // The graph of every B_{i,n} from AllBernstein: basis[i][s] = (u_s, B_{i,n}(u_s)).
    [[nodiscard]] std::vector<std::vector<glm::dvec2>> SampledBasis() const;
    [[nodiscard]] ImU32 ColorOf(std::size_t i, float alpha = 1.0f) const;
    // True when another B_{i,n} is highlighted, so this one is dimmed or hidden.
    [[nodiscard]] bool IsDimmed(std::size_t i) const;

    int m_degree = 3;
    double m_u0 = 0.5;
    int m_sampleCount = 200;

    // When on, B_{m_highlighted,n} is drawn thick from Bernstein (A1.2) over the dimmed rest.
    bool m_highlight = false;
    int m_highlighted = 0;

    bool m_showPartition = true;

    Viewport2D m_view;
};

} // namespace NURBS::Viewer
