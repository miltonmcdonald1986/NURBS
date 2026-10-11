#include <Graphics/OrbitCamera.hpp>

#include <Graphics/Theme.hpp>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <utility>

namespace Graphics
{

namespace
{

constexpr double kRadiansPerPixel = 0.01;
// Straight up or down; Up keeps the view defined there.
constexpr double kMaxPitch = 0.5 * std::numbers::pi;
constexpr double kMinDistance = 1e-4;
constexpr double kMaxDistance = 1e6;
// The clip planes, as multiples of the distance.
constexpr double kNear = 0.01;
constexpr double kFar = 100.0;

} // namespace

glm::dvec3 OrbitCamera::Eye() const
{
    const double c = std::cos(m_pitch);
    return m_target + m_distance * glm::dvec3(c * std::cos(m_yaw), c * std::sin(m_yaw), std::sin(m_pitch));
}

glm::dvec3 OrbitCamera::Up() const
{
    // The derivative of the eye's direction with respect to pitch.
    const double s = std::sin(m_pitch);
    return {-s * std::cos(m_yaw), -s * std::sin(m_yaw), std::cos(m_pitch)};
}

glm::dmat4 OrbitCamera::View() const
{
    return glm::lookAt(Eye(), m_target, Up());
}

glm::dmat4 OrbitCamera::Projection(double aspect) const
{
    const double far = kFar * m_distance;
    if (m_orthographic)
    {
        // Matches the perspective view's extent at the target's depth.
        const double halfHeight = m_distance * std::tan(0.5 * m_fovY);
        const double halfWidth = halfHeight * aspect;
        return glm::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, -far, far);
    }
    return glm::perspective(m_fovY, aspect, kNear * m_distance, far);
}

void OrbitCamera::Orbit(double dx, double dy)
{
    m_yaw -= dx * kRadiansPerPixel;
    m_pitch = std::clamp(m_pitch + dy * kRadiansPerPixel, -kMaxPitch, kMaxPitch);
}

void OrbitCamera::Pan(double dx, double dy, double viewportHeight)
{
    const glm::dvec3 forward = glm::normalize(m_target - Eye());
    const glm::dvec3 up = Up();
    const glm::dvec3 right = glm::cross(forward, up);

    const double worldPerPixel = 2.0 * m_distance * std::tan(0.5 * m_fovY) / std::max(viewportHeight, 1.0);
    m_target += worldPerPixel * (-dx * right + dy * up);
}

void OrbitCamera::Zoom(double steps)
{
    m_distance = std::clamp(m_distance * std::pow(Theme::kZoomPerWheelStep, -steps), kMinDistance, kMaxDistance);
}

void OrbitCamera::Fit(glm::dvec3 min, glm::dvec3 max, double aspect)
{
    // Give a degenerate (zero-size) box some extent so the distance stays positive.
    constexpr double kMinRadius = 1e-3;
    const double radius = std::max(0.5 * glm::length(max - min), kMinRadius);

    // The bounding sphere fits the narrower of the two fields of view.
    const double halfFovX = std::atan(std::tan(0.5 * m_fovY) * aspect);
    const double halfFov = std::min(0.5 * m_fovY, halfFovX);

    m_target = 0.5 * (min + max);
    m_distance = std::clamp(radius / (Theme::kFitMargin * std::sin(halfFov)), kMinDistance, kMaxDistance);
}

void OrbitCamera::SetView(StandardView view)
{
    using std::numbers::pi;
    struct Angles
    {
        double yaw;
        double pitch;
    };
    // The front view's yaw, from which the top and bottom views tilt.
    constexpr double kFrontYaw = -0.5 * pi;
    // Indexed by StandardView, in its declaration order.
    const std::array<Angles, 7> kViews{{
        {kFrontYaw, kMaxPitch},  // Top
        {kFrontYaw, -kMaxPitch}, // Bottom
        {kFrontYaw, 0.0},        // Front
        {0.5 * pi, 0.0},         // Back
        {pi, 0.0},               // Left
        {0.0, 0.0},              // Right
        // The eye along (1, -1, 1), with the front, right, and top in view.
        {-0.25 * pi, std::atan(1.0 / std::numbers::sqrt2)}, // Isometric
    }};
    const Angles& angles = kViews[std::to_underlying(view)];
    m_yaw = angles.yaw;
    m_pitch = angles.pitch;
}

} // namespace Graphics
