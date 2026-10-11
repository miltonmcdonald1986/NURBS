#pragma once

#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <array>
#include <cstddef>

namespace Graphics::Detail
{

// A world axis as the corner gnomon shows it.
struct GnomonAxis
{
    int axis = 0;        // 0 = x, 1 = y, 2 = z
    glm::dvec2 dir{};    // Screen direction, y down; shorter than 1 when foreshortened.
    double depth = 0.0;  // View-space z; larger is closer to the eye.
};

// The world axes seen through view's rotation (its translation is ignored),
// ordered back to front so that drawing in order puts the nearer axes on top.
[[nodiscard]] inline std::array<GnomonAxis, 3> GnomonAxes(const glm::dmat4& view)
{
    const glm::dmat3 rotation(view);
    std::array<GnomonAxis, 3> axes;
    for (int i = 0; i < 3; ++i)
    {
        glm::dvec3 e(0.0);
        e[i] = 1.0;
        const glm::dvec3 v = rotation * e;
        axes[static_cast<std::size_t>(i)] = {i, {v.x, -v.y}, v.z};
    }
    std::sort(axes.begin(), axes.end(), [](const GnomonAxis& a, const GnomonAxis& b) { return a.depth < b.depth; });
    return axes;
}

} // namespace Graphics::Detail
