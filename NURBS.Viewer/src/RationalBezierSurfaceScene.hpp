#pragma once

#include "SurfaceView.hpp"

#include <Graphics/Scene.hpp>

#include <NURBS/bezier.hpp>

#include <glm/vec3.hpp>

#include <cstddef>

namespace NURBS::Viewer
{

// Illustrates NURBS::RationalBezierSurface (Eq. 4.15): a rational
// tensor-product Bezier surface
// S(u, v) = sum B_{i,n}(u) B_{j,m}(v) w[i][j] P[i][j] / sum B_{i,n}(u) B_{j,m}(v) w[i][j],
// whose control net can be dragged in 3D, whose weights can be changed with
// the mouse wheel over a point, and the point S(u0, v0) on it. The polynomial
// surface on the same net (all weights 1) can be shown for comparison.
class RationalBezierSurfaceScene final : public Graphics::Scene
{
public:
    RationalBezierSurfaceScene();

    [[nodiscard]] const char* Name() const override { return "Rational Bezier surface (Eq. 4.15)"; }
    void DrawUI() override;
    void DrawViewport() override;
    void DrawViewOptions() override;

private:
    void LoadPreset(std::size_t preset);
    void SetDegrees(int n, int m);
    void FitView();
    // The selected point's coordinates and weight, then every point's in a collapsed header.
    void DrawControlPointsUI();
    // Scales the weight of the point under the mouse by the wheel's movement.
    void WheelWeight();
    // Isocurves of the polynomial surface on the same net.
    void DrawPolynomialSurface();

    // Its control points and weights are edited in place; only a degree change
    // or a preset builds a new surface.
    NURBS::RationalBezierSurface<glm::dvec3> m_surface;
    std::size_t m_preset = 0;
    bool m_showPolynomial = false;

    SurfaceView m_view;
};

} // namespace NURBS::Viewer
