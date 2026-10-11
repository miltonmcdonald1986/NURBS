#pragma once

#include "Scene.hpp"
#include "Viewport3D.hpp"

#include <glm/vec3.hpp>

#include <cstddef>
#include <vector>

namespace NURBS::Viewer
{

// Illustrates Algorithm A1.6 (horner2): a power basis surface
// S(u, v) = sum a[i][j] * u^i * v^j, i = 0..n, j = 0..m, drawn as a grid of
// isoparametric curves, with the point S(u0, v0).
class PowerBasisSurfaceScene final : public Scene
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
    std::size_t m_preset = 0;

    Viewport3D m_view;
};

} // namespace NURBS::Viewer
