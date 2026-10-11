#pragma once

#include <imgui.h>

// The look and navigation feel shared by Viewport2D and Viewport3D.

namespace Graphics::Theme
{

inline constexpr ImU32 kBackgroundColor = IM_COL32(24, 24, 28, 255);
inline constexpr ImU32 kGridColor = IM_COL32(48, 48, 56, 255);
// Viewport2D draws both axes in one color; Viewport3D colors them x red, y green, z blue.
inline constexpr ImU32 kAxisColor = IM_COL32(110, 110, 125, 255);
inline constexpr ImU32 kXAxisColor = IM_COL32(200, 80, 80, 255);
inline constexpr ImU32 kYAxisColor = IM_COL32(100, 190, 90, 255);
inline constexpr ImU32 kZAxisColor = IM_COL32(90, 140, 240, 255);
// Behind labels drawn over a GL image, to keep them readable.
inline constexpr ImU32 kLabelShadowColor = IM_COL32(0, 0, 0, 200);

// How much one mouse wheel notch zooms.
inline constexpr double kZoomPerWheelStep = 1.15;
// The share of the canvas a fitted box fills.
inline constexpr double kFitMargin = 0.85;

} // namespace Graphics::Theme
