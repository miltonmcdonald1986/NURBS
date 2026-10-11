#pragma once

#include <Graphics/TriangleMesh.hpp>
#include <Graphics/Viewport2D.hpp>
#include <Graphics/Viewport3D.hpp>

#include <glm/vec3.hpp>

#include <cstddef>
#include <functional>
#include <span>
#include <vector>

namespace NURBS::Viewer
{

// The 3D view shared by the Bezier and rational Bezier surface scenes: a
// surface S(u, v), u and v in [0, 1], drawn as a shaded mesh with a grid of
// isocurves, its control net P[i][j] with points that can be dragged, and the
// point S(u0, v0) with the isocurve S(u, v0) highlighted.
//
// A dragged point shows where it was grabbed, a line down to the z = 0 grid
// with its shadow there, and a guide for its constraint, so its position in
// depth stays readable. Clicking a point also selects it, for the scenes to
// edit precisely in their panels.
//
// A square inset in the panel shows the parameter domain [0, 1]^2: grid lines
// at the isocurves' parameters, the line v = v0, and (u0, v0), which can be
// dragged. Each control point P[i][j] is marked at (i / n, j / m), where its
// basis function B_{i,n}(u) B_{j,m}(v) peaks. For the point in focus (under
// the mouse in either view, else the selected one) the domain is shaded by
// how strongly that point pulls on S(u, v) there, i.e. which part of the
// surface moves when it is dragged.
//
// Each frame, DrawUI's sections in the panel, DrawViewOptions in the scene's
// View overlay, then in the viewport:
// @code
//     if (view.Begin("id", points)) ... store the moved points ...
//     view.Draw(surface, points, columns);
//     view.End();
// @endcode
class SurfaceView
{
public:
    // The surface, as a function of (u, v).
    using Surface = std::function<glm::dvec3(double, double)>;

    // Marks the mesh for rebuilding, after the surface changed other than by
    // a drag in Begin.
    void Invalidate() { m_meshDirty = true; }
    // Ends any drag and clears the selection, e.g. when the net is replaced.
    void ClearSelection();
    // Frames the control net's bounding box, which by the convex hull
    // property holds the surface (for a rational one, with positive weights).
    // Precondition: points is non-empty.
    void FitTo(std::span<const glm::dvec3> points);

    SurfaceView();

    // The u0 and v0 sliders, with their animations.
    void DrawParameterUI();
    // The domain inset, under a "Domain" header that starts open, for a net
    // of rows x columns points. When weights is
    // non-empty (one per point), the shading is the rational basis function
    // R_{i,j}(u, v) = B_{i,n}(u) B_{j,m}(v) w[i][j] / sum B_{k,n}(u) B_{l,m}(v) w[k][l].
    void DrawDomainUI(std::size_t rows, std::size_t columns, std::span<const double> weights = {});
    // What is shown: the shaded surface, isocurves, control net, labels.
    void DrawDisplayUI();
    // The graphics-only options, for the App's View overlay: translucency,
    // mesh resolution, samples, projection, standard views, and "Fit view" with the navigation
    // and drag hints. Returns true when Fit view was clicked.
    [[nodiscard]] bool DrawViewOptions();

    [[nodiscard]] double U0() const { return m_u0; }
    [[nodiscard]] double V0() const { return m_v0; }
    // The index into the flat net of the selected point, or -1.
    [[nodiscard]] int Selected() const { return m_selected; }
    // The index of the point being dragged, else of the point under the mouse, or -1.
    [[nodiscard]] int PointUnderMouse() const;

    // Starts the viewport and lets a point of the flat row-major net be
    // dragged. Returns true if a point moved, which also invalidates the mesh.
    bool Begin(const char* id, std::span<glm::dvec3> points);
    // This frame's wheel movement over the viewport, which then doesn't zoom.
    float TakeMouseWheel() { return m_view.TakeMouseWheel(); }
    // The viewport, between Begin and End, to draw more into.
    [[nodiscard]] Graphics::Viewport3D& View() { return m_view; }
    // Draws the surface and the net with columns points per row. When weights
    // is non-empty (one per point), each point's size and label show its weight.
    void Draw(const Surface& surface, std::span<const glm::dvec3> points, std::size_t columns,
              std::span<const double> weights = {});
    void End() { m_view.End(); }

private:
    [[nodiscard]] bool Dragging() const { return m_drag.active >= 0; }
    // The point whose influence is shown: the one under the mouse in the 3D
    // view, else in the domain inset, else the selected one; -1 for none.
    [[nodiscard]] int FocusPoint() const;
    void DrawDomain(std::size_t rows, std::size_t columns, std::span<const double> weights);
    // Shades the domain by the basis function of point k, normalized to its
    // peak, and returns that peak.
    double DrawInfluence(std::size_t k, std::size_t rows, std::size_t columns, std::span<const double> weights) const;
    // Rebuilds the mesh and isocurves if the surface changed, coarser while dragging.
    void UpdateMesh(const Surface& surface);
    void DrawControlNet(std::span<const glm::dvec3> points, std::size_t columns);
    void DrawControlPoints(std::span<const glm::dvec3> points, std::size_t columns, std::span<const double> weights);
    // The ghost, drop line, shadow, and constraint guide of the dragged point.
    void DrawDragCues(std::span<const glm::dvec3> points);

    double m_u0 = 0.3;
    double m_v0 = 0.6;
    // u0 wraps from 1 back to 0; v0 bounces between 0 and 1, moving in the
    // direction of m_vDirection's sign.
    bool m_animateU = false;
    bool m_animateV = false;
    double m_vDirection = 1.0;
    float m_sweepSeconds = 4.0f;

    bool m_showSurface = true;
    bool m_translucentSurface = false;
    float m_surfaceOpacity = 0.5f;
    int m_meshResolution = 48;
    bool m_showIsocurves = true;
    int m_isocurveCount = 11;
    int m_sampleCount = 48;
    bool m_showNet = true;
    bool m_labelAllPoints = false;

    Graphics::Viewport2D m_domainView;
    bool m_showInfluence = true;
    // (u0, v0) being dragged in the inset (0) or not (-1).
    int m_domainDragged = -1;
    // The control point whose mark is under the mouse in the inset, or -1.
    int m_domainHovered = -1;
    // The point last shaded in the inset and its influence's largest value,
    // for the caption under it; -1 when none was.
    int m_influencePoint = -1;
    double m_influencePeak = 0.0;

    Graphics::Viewport3D m_view;
    Graphics::DragPointsState m_drag;
    int m_selected = -1;
    // The net's extent, for sizing the drag guides.
    double m_extent = 1.0;

    // Rebuilt by UpdateMesh, with the isocurves, when dirty or when the
    // resolution wanted changes (as a drag starts or ends).
    Graphics::TriangleMesh m_mesh;
    std::vector<std::vector<glm::dvec3>> m_isocurves;
    bool m_meshDirty = true;
    int m_builtResolution = 0;
};

} // namespace NURBS::Viewer
