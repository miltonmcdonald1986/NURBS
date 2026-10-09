#pragma once

#include "Scene.hpp"
#include "Viewport2D.hpp"

#include <glm/vec2.hpp>

#include <cstddef>
#include <vector>

namespace NURBS::Viewer
{

// Illustrates Algorithm A1.1 (horner1): a 2D power basis curve
// C(u) = sum a[i] * u^i, i = 0..n, and the Horner chain that evaluates it at u0:
// C_n = a[n], C_k = u0 * C_{k+1} + a[k], ending at C_0 = C(u0).
class HornerScene final : public Scene
{
public:
    HornerScene();

    [[nodiscard]] const char* Name() const override { return "Horner (A1.1)"; }
    void DrawUI() override;
    void DrawViewport() override;

private:
    void LoadPreset(std::size_t preset);
    void FitView();
    void FitCoefficientView();
    void DrawCoefficientView();
    void DrawStepsTable(const std::vector<glm::dvec2>& chain) const;

    // The Horner chain at m_u0: chain[k] = C_k for k = 0..n.
    [[nodiscard]] std::vector<glm::dvec2> HornerChain() const;
    // Partial sums of the power basis terms at m_u0: sums[i] = sum a[j] * u0^j, j = 0..i.
    [[nodiscard]] std::vector<glm::dvec2> PowerSum() const;
    [[nodiscard]] std::vector<glm::dvec2> SampledCurve() const;

    std::vector<glm::dvec2> m_coefficients;
    double m_uMin = 0.0;
    double m_uMax = 1.0;
    double m_u0 = 0.5;
    int m_sampleCount = 200;
    std::size_t m_preset = 0;

    bool m_showChain = true;
    // Number of Horner steps drawn, 0..n: C_n down to C_{n - m_stepsShown}.
    int m_stepsShown = 0;

    bool m_showPowerSum = false;

    Viewport2D m_view;

    // The side panel's subview of the coefficients as vectors from a common
    // origin, whose tips can be dragged; -1 when none is being dragged.
    Viewport2D m_coefficientView;
    int m_draggedCoefficient = -1;
};

} // namespace NURBS::Viewer
