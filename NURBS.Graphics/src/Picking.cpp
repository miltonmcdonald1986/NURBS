#include <Graphics/Picking.hpp>

#include <glm/geometric.hpp>
#include <glm/vec4.hpp>

#include <cmath>

namespace Graphics
{

namespace
{

constexpr glm::dvec3 kUp{0.0, 0.0, 1.0};

} // namespace

Ray ScreenRay(const glm::dmat4& inverseViewProjection, glm::dvec2 ndc)
{
    // The points under ndc on the near (z = -1) and far (z = 1) planes.
    const glm::dvec4 nearClip = inverseViewProjection * glm::dvec4(ndc, -1.0, 1.0);
    const glm::dvec4 farClip = inverseViewProjection * glm::dvec4(ndc, 1.0, 1.0);
    const glm::dvec3 nearPoint = glm::dvec3(nearClip) / nearClip.w;
    const glm::dvec3 farPoint = glm::dvec3(farClip) / farClip.w;
    return {nearPoint, glm::normalize(farPoint - nearPoint)};
}

std::optional<glm::dvec3> IntersectPlane(const Ray& ray, glm::dvec3 point, glm::dvec3 normal, double minCosine)
{
    const double cosine = glm::dot(ray.direction, normal);
    if (std::abs(cosine) < minCosine)
        return std::nullopt;
    return ray.At(glm::dot(point - ray.origin, normal) / cosine);
}

std::optional<glm::dvec3> NearestPointOnLine(const Ray& ray, glm::dvec3 point, glm::dvec3 direction, double minSine)
{
    // Minimize |ray.At(t) - (point + s * direction)| over t and s; with unit
    // directions and c = cos of the angle between them, the normal equations give
    // s = (d.w - c * r.w) / (1 - c^2), where w = ray.origin - point.
    const double c = glm::dot(ray.direction, direction);
    const double sineSquared = 1.0 - c * c;
    if (sineSquared < minSine * minSine)
        return std::nullopt;

    const glm::dvec3 w = ray.origin - point;
    const double s = (glm::dot(direction, w) - c * glm::dot(ray.direction, w)) / sineSquared;
    return point + s * direction;
}

void PointDrag::Anchor(glm::dvec3 p, DragConstraint constraint, const Ray& ray, glm::dvec3 viewDirection)
{
    m_anchor = p;
    m_constraint = constraint;
    m_axis = constraint == DragConstraint::ViewPlane ? viewDirection : kUp;
    m_fellBack = false;

    std::optional<glm::dvec3> hit = Hit(ray);
    if (!hit && constraint != DragConstraint::ViewPlane)
    {
        // Seen too nearly edge-on (or end-on, for the vertical line) to follow;
        // the view plane always faces the camera.
        m_constraint = DragConstraint::ViewPlane;
        m_axis = viewDirection;
        m_fellBack = true;
        hit = Hit(ray);
    }
    m_offset = hit ? p - *hit : glm::dvec3{0.0};
}

std::optional<glm::dvec3> PointDrag::Follow(const Ray& ray) const
{
    const std::optional<glm::dvec3> hit = Hit(ray);
    if (!hit)
        return std::nullopt;
    return *hit + m_offset;
}

std::optional<glm::dvec3> PointDrag::Hit(const Ray& ray) const
{
    if (m_constraint == DragConstraint::Vertical)
        return NearestPointOnLine(ray, m_anchor, m_axis);
    return IntersectPlane(ray, m_anchor, m_axis);
}

} // namespace Graphics
