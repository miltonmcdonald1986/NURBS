/// @file
/// OrbitCamera, a turntable camera.

#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace Graphics
{

/// A turntable camera orbiting a target point, with z up. The eye sits at
/// distance from the target, at angle yaw around the z axis (from +x) and
/// pitch above the xy plane. Pure math: no GL or ImGui.
class OrbitCamera
{
public:
    /// The point the camera orbits.
    /// @return The target point.
    [[nodiscard]] glm::dvec3 Target() const { return m_target; }
    /// The distance from the eye to the target.
    /// @return The distance.
    [[nodiscard]] double Distance() const { return m_distance; }
    /// The camera position.
    /// @return The eye point.
    [[nodiscard]] glm::dvec3 Eye() const;

    /// Whether the projection is orthographic.
    /// @return True for orthographic, false for perspective.
    [[nodiscard]] bool IsOrthographic() const { return m_orthographic; }
    /// Switches between orthographic and perspective projection.
    /// @param orthographic True for orthographic, false for perspective.
    void SetOrthographic(bool orthographic) { m_orthographic = orthographic; }

    /// The view matrix.
    /// @return The world-to-eye transform.
    [[nodiscard]] glm::dmat4 View() const;
    /// The projection matrix.
    /// @param aspect The viewport's width / height.
    /// @return The eye-to-clip transform.
    [[nodiscard]] glm::dmat4 Projection(double aspect) const;

    /// Rotates the eye around the target, driven by the mouse.
    /// @param dx The horizontal mouse delta, in pixels.
    /// @param dy The vertical mouse delta, in pixels, with y down as on screen.
    void Orbit(double dx, double dy);
    /// Moves the target in the view plane so that a point at the target's
    /// depth follows the mouse.
    /// @param dx The horizontal mouse delta, in pixels.
    /// @param dy The vertical mouse delta, in pixels, with y down as on screen.
    /// @param viewportHeight The viewport height, in the same pixels as @p dx and @p dy.
    void Pan(double dx, double dy, double viewportHeight);
    /// Moves the eye toward the target for positive steps.
    /// @param steps The number of mouse wheel notches.
    void Zoom(double steps);

    /// Centers the box [@p min, @p max] and backs off until it fits the view.
    /// @param min The box's minimum corner.
    /// @param max The box's maximum corner.
    /// @param aspect The viewport's width / height.
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
