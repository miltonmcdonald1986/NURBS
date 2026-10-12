#include <Graphics/PointDrag.hpp>

#include <optional>

namespace Graphics
{

namespace
{

constexpr glm::dvec3 kUp{0.0, 0.0, 1.0};

} // namespace

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
