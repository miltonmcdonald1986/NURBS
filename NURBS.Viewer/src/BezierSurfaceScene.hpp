#pragma once

#include "SurfaceView.hpp"

#include <Graphics/Scene.hpp>

#include <NURBS/bezier_surface.hpp>

#include <glm/vec3.hpp>

#include <cstddef>

namespace NURBS::Viewer
{

// Illustrates NURBS::BezierSurface and Algorithm A1.7 (deCasteljau2): a
// tensor-product Bezier surface S(u, v) = sum B_{i,n}(u) B_{j,m}(v) P[i][j],
// whose control net can be dragged in 3D, and the point S(u0, v0) on it.
class BezierSurfaceScene final : public Graphics::Scene
{
public:
    BezierSurfaceScene();

    [[nodiscard]] const char* Name() const override { return "Bezier surface (A1.7)"; }
    void DrawUI() override;
    void DrawViewport() override;
    void DrawViewOptions() override;

private:
    void LoadPreset(std::size_t preset);
    void SetDegrees(int n, int m);
    void FitView();
    // The selected point's coordinates, then every point's in a collapsed header.
    void DrawControlPointsUI();

    // Its control points are dragged and edited in place; only a degree change
    // or a preset builds a new surface.
    NURBS::BezierSurface<glm::dvec3> m_surface;
    std::size_t m_preset = 0;

    SurfaceView m_view;
};

} // namespace NURBS::Viewer
