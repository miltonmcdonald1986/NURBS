#pragma once

#include <Graphics/Bounds.hpp>
#include <Graphics/Scene.hpp>
#include <Graphics/TriangleMesh.hpp>
#include <Graphics/Viewport3D.hpp>

#include <glm/vec3.hpp>

#include <cstddef>
#include <vector>

namespace NURBS::Viewer
{

// Illustrates Algorithm A1.6 (horner2): a power basis surface
// S(u, v) = sum a[i][j] * u^i * v^j, i = 0..n, j = 0..m, drawn as a shaded
// mesh with a grid of isoparametric curves, and the point S(u0, v0). The
// parameter domain is drawn as a grid on the floor below the surface. The
// isocurve S(u, v0) is highlighted, with its part from uMin to u0 brighter,
// and a line runs from (u0, v0) on the floor up to S(u0, v0). u0 and v0 can
// be animated.
class PowerBasisSurfaceScene final : public Graphics::Scene
{
public:
    PowerBasisSurfaceScene();

    [[nodiscard]] const char* Name() const override { return "Power basis surface (A1.6)"; }
    void DrawUI() override;
    void DrawViewport() override;

private:
    void LoadPreset(std::size_t preset);
    void FitView();
    // Resizes the coefficient grid to (n + 1) x (m + 1), keeping the
    // coefficients that remain and padding with zeros.
    void Resize(std::size_t n, std::size_t m);

    [[nodiscard]] glm::dvec3 Evaluate(double u, double v) const;
    // Curves of constant u (sampled along v) followed by curves of constant v
    // (sampled along u), m_isocurveCount of each.
    [[nodiscard]] std::vector<std::vector<glm::dvec3>> SampledIsocurves() const;
    // The bounding box of SampledIsocurves().
    [[nodiscard]] Graphics::Bounds3D SurfaceBounds() const;
    // S(u, v) for u from m_uMin to uEnd, at samples points (at least 2).
    [[nodiscard]] std::vector<glm::dvec3> Isocurve(double v, double uEnd, std::size_t samples) const;

    // Advances u0 and v0 by one frame of their animations.
    void Animate();
    void DrawFloorGrid();

    // a[i][j] multiplies u^i v^j, as in horner2.
    std::vector<std::vector<glm::dvec3>> m_coefficients;
    double m_uMin = -1.0;
    double m_uMax = 1.0;
    double m_vMin = -1.0;
    double m_vMax = 1.0;
    double m_u0 = 0.5;
    double m_v0 = 0.5;
    int m_isocurveCount = 11;
    int m_sampleCount = 64;
    int m_meshResolution = 64;
    bool m_showSurface = true;
    bool m_showIsocurves = true;
    std::size_t m_preset = 0;

    // u0 wraps from m_uMax back to m_uMin; v0 bounces between m_vMin and
    // m_vMax, moving in the direction of m_vDirection's sign.
    bool m_animateU = false;
    bool m_animateV = false;
    double m_vDirection = 1.0;
    // Seconds for one pass across the domain.
    float m_sweepSeconds = 4.0f;

    Graphics::Viewport3D m_view;
    // Rebuilt in DrawViewport when the surface or its resolution changes, with
    // m_floorZ, the height of the plane the domain grid is drawn in.
    Graphics::TriangleMesh m_mesh;
    bool m_meshDirty = true;
    double m_floorZ = 0.0;
};

} // namespace NURBS::Viewer
