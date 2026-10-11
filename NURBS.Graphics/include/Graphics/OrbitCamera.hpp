#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace Graphics
{

// A turntable camera orbiting a target point, with z up. The eye sits at
// distance from the target, at angle yaw around the z axis (from +x) and
// pitch above the xy plane. Pure math: no GL or ImGui.
class OrbitCamera
{
public:
    [[nodiscard]] glm::dvec3 Target() const { return m_target; }
    [[nodiscard]] double Distance() const { return m_distance; }
    [[nodiscard]] glm::dvec3 Eye() const;

    [[nodiscard]] bool IsOrthographic() const { return m_orthographic; }
    void SetOrthographic(bool orthographic) { m_orthographic = orthographic; }

    [[nodiscard]] glm::dmat4 View() const;
    // aspect is the viewport's width / height.
    [[nodiscard]] glm::dmat4 Projection(double aspect) const;

    // Mouse-driven moves; deltas are in pixels, with y down as on screen.
    void Orbit(double dx, double dy);
    // Moves the target in the view plane so that a point at the target's depth
    // follows the mouse; viewportHeight is in the same pixels as dx and dy.
    void Pan(double dx, double dy, double viewportHeight);
    // Moves the eye toward the target for positive steps (wheel notches).
    void Zoom(double steps);

    // Centers the box [min, max] and backs off until it fits the view.
    void Fit(glm::dvec3 min, glm::dvec3 max, double aspect);

private:
    glm::dvec3 m_target{0.0, 0.0, 0.0};
    double m_distance = 5.0;
    double m_yaw = -1.0471975511965976; // -60 degrees
    double m_pitch = 0.5;
    double m_fovY = 0.7853981633974483; // 45 degrees
    bool m_orthographic = false;
};

} // namespace Graphics
