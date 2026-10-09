#include "Viewport2D.hpp"

#include <glm/common.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace NURBS::Viewer
{

namespace
{

constexpr double kMinPixelsPerUnit = 1e-3;
constexpr double kMaxPixelsPerUnit = 1e7;
constexpr double kZoomPerWheelStep = 1.15;
constexpr double kFitMargin = 0.85;
constexpr float kMinGridSpacingPixels = 40.0f;

constexpr ImU32 kBackgroundColor = IM_COL32(24, 24, 28, 255);
constexpr ImU32 kGridColor = IM_COL32(48, 48, 56, 255);
constexpr ImU32 kAxisColor = IM_COL32(110, 110, 125, 255);

} // namespace

void Viewport2D::Begin(const char* id)
{
    m_drawList = ImGui::GetWindowDrawList();
    m_min = ImGui::GetCursorScreenPos();
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    m_size = ImVec2(std::max(avail.x, 1.0f), std::max(avail.y, 1.0f));

    ImGui::InvisibleButton(id, m_size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    m_hovered = ImGui::IsItemHovered();
    // Claim the wheel so zooming doesn't also scroll an enclosing window.
    if (m_hovered)
        ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);

    ApplyPendingFit();
    HandleInput();

    const ImVec2 max(m_min.x + m_size.x, m_min.y + m_size.y);
    m_drawList->PushClipRect(m_min, max, true);
    m_drawList->AddRectFilled(m_min, max, kBackgroundColor);
    DrawGrid();
}

void Viewport2D::End()
{
    assert(m_drawList != nullptr);
    m_drawList->PopClipRect();
    m_drawList = nullptr;
}

void Viewport2D::FitTo(glm::dvec2 min, glm::dvec2 max)
{
    m_pendingFit = Box{min, max};
}

ImVec2 Viewport2D::ToScreen(glm::dvec2 p) const
{
    const glm::dvec2 offset = (p - m_center) * m_pixelsPerUnit;
    return {static_cast<float>(m_min.x + 0.5 * m_size.x + offset.x), static_cast<float>(m_min.y + 0.5 * m_size.y - offset.y)};
}

glm::dvec2 Viewport2D::ToWorld(ImVec2 p) const
{
    const double dx = static_cast<double>(p.x) - (m_min.x + 0.5 * m_size.x);
    const double dy = (m_min.y + 0.5 * m_size.y) - static_cast<double>(p.y);
    return m_center + glm::dvec2(dx, dy) / m_pixelsPerUnit;
}

glm::dvec2 Viewport2D::MouseWorld() const
{
    return ToWorld(ImGui::GetIO().MousePos);
}

bool Viewport2D::DragPoints(std::span<glm::dvec2> points, int& active, float radius) const
{
    if (active >= static_cast<int>(points.size()))
        active = -1;

    if (active < 0 && m_hovered)
    {
        // The point nearest the cursor, if it is within radius.
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        float bestDistanceSquared = radius * radius;
        int nearest = -1;
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const ImVec2 s = ToScreen(points[i]);
            const float dx = s.x - mouse.x;
            const float dy = s.y - mouse.y;
            if (dx * dx + dy * dy <= bestDistanceSquared)
            {
                bestDistanceSquared = dx * dx + dy * dy;
                nearest = static_cast<int>(i);
            }
        }
        if (nearest >= 0)
        {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                active = nearest;
        }
    }

    if (active < 0)
        return false;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
    {
        active = -1;
        return false;
    }

    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    const glm::dvec2 mouse = MouseWorld();
    glm::dvec2& point = points[static_cast<std::size_t>(active)];
    const bool moved = point != mouse;
    point = mouse;
    return moved;
}

void Viewport2D::Polyline(std::span<const glm::dvec2> points, ImU32 color, float thickness) const
{
    if (points.size() < 2)
        return;

    ImVector<ImVec2> screen;
    screen.reserve(static_cast<int>(points.size()));
    for (const glm::dvec2& p : points)
        screen.push_back(ToScreen(p));
    m_drawList->AddPolyline(screen.Data, screen.Size, color, ImDrawFlags_None, thickness);
}

void Viewport2D::Line(glm::dvec2 a, glm::dvec2 b, ImU32 color, float thickness) const
{
    m_drawList->AddLine(ToScreen(a), ToScreen(b), color, thickness);
}

void Viewport2D::Arrow(glm::dvec2 from, glm::dvec2 to, ImU32 color, float thickness) const
{
    constexpr float kHeadLength = 10.0f;
    constexpr float kHeadHalfWidth = 4.5f;

    const ImVec2 a = ToScreen(from);
    const ImVec2 b = ToScreen(to);
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length < 1.0f)
        return;

    // Shorten the shaft so it doesn't poke through the head; shrink the head
    // for arrows shorter than it.
    const float head = std::min(kHeadLength, 0.5f * length);
    const float halfWidth = kHeadHalfWidth * head / kHeadLength;
    const float ux = dx / length;
    const float uy = dy / length;
    const ImVec2 base(b.x - ux * head, b.y - uy * head);

    m_drawList->AddLine(a, base, color, thickness);
    m_drawList->AddTriangleFilled(b, ImVec2(base.x - uy * halfWidth, base.y + ux * halfWidth),
                                  ImVec2(base.x + uy * halfWidth, base.y - ux * halfWidth), color);
}

void Viewport2D::Point(glm::dvec2 p, ImU32 color, float radius) const
{
    m_drawList->AddCircleFilled(ToScreen(p), radius, color);
}

void Viewport2D::Rect(glm::dvec2 a, glm::dvec2 b, ImU32 color) const
{
    // The y axis flips on screen, so sort the corners after converting.
    const ImVec2 sa = ToScreen(a);
    const ImVec2 sb = ToScreen(b);
    m_drawList->AddRectFilled(ImVec2(std::min(sa.x, sb.x), std::min(sa.y, sb.y)),
                              ImVec2(std::max(sa.x, sb.x), std::max(sa.y, sb.y)), color);
}

void Viewport2D::Label(glm::dvec2 p, const char* text, ImU32 color) const
{
    const ImVec2 s = ToScreen(p);
    m_drawList->AddText(ImVec2(s.x + 6.0f, s.y - ImGui::GetFontSize() - 2.0f), color, text);
}

void Viewport2D::ApplyPendingFit()
{
    if (!m_pendingFit)
        return;

    // Give a degenerate (zero-size) box some extent so the zoom stays finite.
    constexpr double kMinExtent = 1e-3;
    const glm::dvec2 extent = glm::max(m_pendingFit->max - m_pendingFit->min, glm::dvec2(kMinExtent));

    m_center = 0.5 * (m_pendingFit->min + m_pendingFit->max);
    m_pixelsPerUnit = kFitMargin * std::min(m_size.x / extent.x, m_size.y / extent.y);
    m_pixelsPerUnit = std::clamp(m_pixelsPerUnit, kMinPixelsPerUnit, kMaxPixelsPerUnit);
    m_pendingFit.reset();
}

void Viewport2D::HandleInput()
{
    const ImGuiIO& io = ImGui::GetIO();

    if (ImGui::IsItemActive() && (ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle)))
        m_center -= glm::dvec2(io.MouseDelta.x, -io.MouseDelta.y) / m_pixelsPerUnit;

    if (m_hovered && io.MouseWheel != 0.0f)
    {
        // Zoom about the cursor: keep the world point under it fixed.
        const glm::dvec2 before = MouseWorld();
        m_pixelsPerUnit *= std::pow(kZoomPerWheelStep, static_cast<double>(io.MouseWheel));
        m_pixelsPerUnit = std::clamp(m_pixelsPerUnit, kMinPixelsPerUnit, kMaxPixelsPerUnit);
        m_center += before - MouseWorld();
    }
}

void Viewport2D::DrawGrid() const
{
    // The smallest power of ten whose spacing on screen is at least kMinGridSpacingPixels.
    const double step = std::pow(10.0, std::ceil(std::log10(kMinGridSpacingPixels / m_pixelsPerUnit)));

    const glm::dvec2 lo = ToWorld(ImVec2(m_min.x, m_min.y + m_size.y));
    const glm::dvec2 hi = ToWorld(ImVec2(m_min.x + m_size.x, m_min.y));

    const auto first = [step](double v) { return static_cast<long long>(std::floor(v / step)); };
    const auto last = [step](double v) { return static_cast<long long>(std::ceil(v / step)); };

    for (long long i = first(lo.x); i <= last(hi.x); ++i)
    {
        const double x = static_cast<double>(i) * step;
        Line({x, lo.y}, {x, hi.y}, i == 0 ? kAxisColor : kGridColor);
    }
    for (long long j = first(lo.y); j <= last(hi.y); ++j)
    {
        const double y = static_cast<double>(j) * step;
        Line({lo.x, y}, {hi.x, y}, j == 0 ? kAxisColor : kGridColor);
    }
}

} // namespace NURBS::Viewer
