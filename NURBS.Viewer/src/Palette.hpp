#pragma once

#include <imgui.h>

// Colours and marker sizes shared by the viewer's scenes, so they look alike.

namespace NURBS::Viewer
{

// A curve being illustrated.
constexpr ImU32 kCurveColor = IM_COL32(90, 170, 255, 255);
// The point a scene evaluates, e.g. C(u0) or S(u0, v0).
constexpr ImU32 kResultColor = IM_COL32(255, 90, 90, 255);
// Light grey for labels and neutral marks.
constexpr ImU32 kNeutralColor = IM_COL32(230, 230, 230, 255);

// The radius of a draggable point, larger while it is dragged.
constexpr float kPointRadius = 4.0f;
constexpr float kDraggedPointRadius = 6.0f;

[[nodiscard]] constexpr float PointRadius(bool isDragged)
{
    return isDragged ? kDraggedPointRadius : kPointRadius;
}

// The radius of the result point.
constexpr float kResultRadius = 6.0f;

// The result point p, labeled, in view (a Graphics::Viewport2D or Viewport3D).
template <typename View, typename Point>
void DrawResultPoint(View& view, Point p, const char* label)
{
    view.LabeledPoint(p, label, kResultColor, kResultRadius);
}

} // namespace NURBS::Viewer
