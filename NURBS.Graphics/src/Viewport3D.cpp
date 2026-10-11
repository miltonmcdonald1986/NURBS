#include <Graphics/Viewport3D.hpp>

#include <Graphics/Theme.hpp>

#include "Gnomon.hpp"
#include "Grid.hpp"
#include "Viewport3DShaders.hpp"

#include <glad/gl.h>
#include <glm/geometric.hpp>
#include <glm/mat3x3.hpp>
#include <glm/matrix.hpp>
#include <glm/vec2.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace Graphics
{

namespace
{

constexpr int kGridHalfLines = 10;
constexpr double kGridLinesPerDistance = 8.0;

// How far lines and points are lifted toward the eye, in multiples of their
// size (a line's thickness, a point's radius). A full thickness keeps lines on
// a surface seen nearly edge-on from breaking up.
constexpr float kLineLift = 1.0f;
constexpr float kPointLift = 1.0f;

// The corner gnomon, in pixels: the length of an axis seen side on, and the
// gap between its reach and the canvas' left and bottom edges.
constexpr float kGnomonLength = 40.0f;
constexpr float kGnomonMargin = 16.0f;
constexpr float kGnomonThickness = 2.0f;
constexpr ImU32 kGnomonBackdropColor = IM_COL32(0, 0, 0, 90);

// Points this many pixels or less nearer the mouse than another count as
// about as near, and the one nearer the eye is picked.
constexpr float kPickTiePixels = 2.0f;

// Signed distance-like value of a clip-space point from the near plane
// (z = -w in GL clip space); non-negative means in front of it.
[[nodiscard]] double NearPlaneSide(glm::dvec4 clip)
{
    return clip.z + clip.w;
}

} // namespace

void Viewport3D::Begin(const char* id)
{
    m_canvas = BeginCanvas(id);
    m_canvasActive = ImGui::IsItemActive();
    m_mouseClaimed = false;
    m_wheelTaken = false;
    ApplyPendingFit();
    UpdateMatrices();
    m_targetBound = BeginRenderTarget();

    m_canvas.drawList->PushClipRect(m_canvas.min, m_canvas.Max(), true);
    if (m_targetBound)
    {
        // Added now so labels land on top; ImGui samples the texture when it
        // renders, after End has resolved into it. GL rows run bottom-up, so flip v.
        const auto texture = static_cast<ImTextureID>(m_target.ColorTexture());
        m_canvas.drawList->AddImage(texture, m_canvas.min, m_canvas.Max(), ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
    }

    m_lines.Clear();
    m_points.Clear();
    m_translucentMeshes.clear();
    DrawGrid();
}

void Viewport3D::End()
{
    assert(m_canvas.drawList != nullptr);
    if (m_targetBound)
    {
        DrawPrimitives();
        DrawTranslucentMeshes();
        m_target.Resolve();
        // App::Run sets the viewport again before ImGui renders.
        glDisable(GL_DEPTH_TEST);
        m_targetBound = false;
    }
    m_lines.Clear();
    m_points.Clear();
    m_translucentMeshes.clear();
    DrawGnomon();
    m_canvas.drawList->PopClipRect();
    m_canvas.drawList = nullptr;

    // Last, so the caller could claim the mouse first. The camera moves for the next frame.
    HandleInput();
}

void Viewport3D::UpdateMatrices()
{
    m_view = m_camera.View();
    m_projection = m_camera.Projection(m_canvas.Aspect());
    m_viewProjection = m_projection * m_view;
    m_inverseViewProjection = glm::inverse(m_viewProjection);
}

bool Viewport3D::BeginRenderTarget()
{
    // The target is in physical pixels, which differ from ImGui's on high-DPI displays.
    const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
    m_pixelScale = scale.y;
    const int width = std::max(1, static_cast<int>(std::lround(m_canvas.size.x * scale.x)));
    const int height = std::max(1, static_cast<int>(std::lround(m_canvas.size.y * scale.y)));
    // A failure is logged once; labels still draw without the GL image.
    if (!m_target.Prepare(width, height))
        return false;

    m_target.Bind();
    glEnable(GL_DEPTH_TEST);
    const glm::vec4 background = ToVec4(Theme::kBackgroundColor);
    glClearColor(background.r, background.g, background.b, background.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    return true;
}

void Viewport3D::DrawMesh(const TriangleMesh& mesh, glm::vec3 frontColor, glm::vec3 backColor, float opacity)
{
    opacity = std::clamp(opacity, 0.0f, 1.0f);
    if (!m_targetBound || mesh.IsEmpty() || opacity <= 0.0f)
        return;

    // Translucent meshes wait for End, so what lies behind them is drawn first.
    if (opacity < 1.0f)
    {
        m_translucentMeshes.push_back({&mesh, frontColor, backColor, opacity});
        return;
    }

    if (!PreparePrograms())
        return;
    RenderMesh(mesh, frontColor, backColor, opacity);
}

void Viewport3D::RenderMesh(const TriangleMesh& mesh, glm::vec3 frontColor, glm::vec3 backColor, float opacity) const
{
    m_meshProgram.Use();
    m_meshProgram.Set("uView", glm::mat4(m_view));
    m_meshProgram.Set("uProjection", glm::mat4(m_projection));
    m_meshProgram.Set("uNormalMatrix", glm::mat3(glm::transpose(glm::inverse(m_view))));
    m_meshProgram.Set("uFrontColor", frontColor);
    m_meshProgram.Set("uBackColor", backColor);
    m_meshProgram.Set("uOpacity", opacity);
    mesh.Draw();
    glUseProgram(0);
}

void Viewport3D::DrawTranslucentMeshes()
{
    if (m_translucentMeshes.empty() || !PreparePrograms())
        return;

    // Depth tested against what is already drawn, but without writing depth,
    // so the layers of a mesh all blend over what lies behind them.
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const TranslucentMesh& m : m_translucentMeshes)
        RenderMesh(*m.mesh, m.frontColor, m.backColor, m.opacity);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Viewport3D::FitTo(glm::dvec3 min, glm::dvec3 max)
{
    m_pendingFit = Bounds3D{min, max};
}

std::optional<ImVec2> Viewport3D::ToScreen(glm::dvec3 p) const
{
    const glm::dvec4 clip = m_viewProjection * glm::dvec4(p, 1.0);
    if (NearPlaneSide(clip) < 0.0 || clip.w <= 0.0)
        return std::nullopt;

    const double x = clip.x / clip.w;
    const double y = clip.y / clip.w;
    return ImVec2(static_cast<float>(m_canvas.min.x + 0.5 * (x + 1.0) * m_canvas.size.x),
                  static_cast<float>(m_canvas.min.y + 0.5 * (1.0 - y) * m_canvas.size.y));
}

glm::dvec3 Viewport3D::ViewDirection() const
{
    return glm::normalize(m_camera.Target() - m_camera.Eye());
}

Ray Viewport3D::MouseRay() const
{
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const double x = 2.0 * (mouse.x - m_canvas.min.x) / m_canvas.size.x - 1.0;
    const double y = 1.0 - 2.0 * (mouse.y - m_canvas.min.y) / m_canvas.size.y;
    return ScreenRay(m_inverseViewProjection, {x, y});
}

int Viewport3D::NearestToMouse(std::span<const glm::dvec3> points, float radius) const
{
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const glm::dvec3 eye = m_camera.Eye();
    const glm::dvec3 forward = ViewDirection();

    int nearest = -1;
    float nearestPixels = radius;
    double nearestDepth = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const std::optional<ImVec2> s = ToScreen(points[i]);
        if (!s)
            continue;
        const float pixels = std::hypot(s->x - mouse.x, s->y - mouse.y);
        if (pixels > radius)
            continue;

        const double depth = glm::dot(points[i] - eye, forward);
        const bool clearlyNearer = pixels < nearestPixels - kPickTiePixels;
        const bool aboutAsNear = std::abs(pixels - nearestPixels) <= kPickTiePixels;
        if (nearest < 0 || clearlyNearer || (aboutAsNear && depth < nearestDepth))
        {
            nearest = static_cast<int>(i);
            nearestPixels = pixels;
            nearestDepth = depth;
        }
    }
    return nearest;
}

bool Viewport3D::DragPoints(std::span<glm::dvec3> points, DragPointsState& state, float radius)
{
    if (state.active >= static_cast<int>(points.size()))
        state.active = -1;
    state.hovered = -1;

    const bool leftDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    if (state.cancelled)
    {
        // Keep the held button from orbiting until it is released.
        state.cancelled = leftDown;
        m_mouseClaimed = leftDown;
        return false;
    }

    const ImGuiIO& io = ImGui::GetIO();
    const auto requested = [&io] {
        if (io.KeyShift)
            return DragConstraint::Vertical;
        if (io.KeyCtrl)
            return DragConstraint::Horizontal;
        return DragConstraint::ViewPlane;
    };

    if (state.active < 0)
    {
        // Only while nothing else (e.g. an orbit) holds the canvas.
        if (!m_canvas.hovered || (m_canvasActive && !ImGui::IsMouseClicked(ImGuiMouseButton_Left)))
            return false;
        state.hovered = NearestToMouse(points, radius);
        if (state.hovered < 0)
            return false;
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
        if (!ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            return false;

        state.active = state.hovered;
        state.start = points[static_cast<std::size_t>(state.active)];
        state.requested = requested();
        state.drag.Anchor(state.start, state.requested, MouseRay(), ViewDirection());
    }

    if (!leftDown)
    {
        state.active = -1;
        return false;
    }

    m_mouseClaimed = true;
    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    glm::dvec3& point = points[static_cast<std::size_t>(state.active)];

    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false))
    {
        const bool moved = point != state.start;
        point = state.start;
        state.active = -1;
        state.cancelled = true;
        return moved;
    }

    // Re-anchor where the point is when the modifiers change, so it doesn't jump.
    if (requested() != state.requested)
    {
        state.requested = requested();
        state.drag.Anchor(point, state.requested, MouseRay(), ViewDirection());
    }

    const std::optional<glm::dvec3> next = state.drag.Follow(MouseRay());
    if (!next || *next == point)
        return false;
    point = *next;
    return true;
}

float Viewport3D::TakeMouseWheel()
{
    if (!m_canvas.hovered)
        return 0.0f;
    m_wheelTaken = true;
    return ImGui::GetIO().MouseWheel;
}

PrimitiveVertex Viewport3D::Vertex(glm::dvec3 p, ImU32 color, float size) const
{
    return {glm::vec3(p), ToVec4(color), size * m_pixelScale};
}

void Viewport3D::Polyline(std::span<const glm::dvec3> points, ImU32 color, float thickness)
{
    for (std::size_t i = 1; i < points.size(); ++i)
        Line(points[i - 1], points[i], color, thickness);
}

void Viewport3D::Line(glm::dvec3 a, glm::dvec3 b, ImU32 color, float thickness)
{
    m_lines.Add(Vertex(a, color, thickness));
    m_lines.Add(Vertex(b, color, thickness));
}

void Viewport3D::Point(glm::dvec3 p, ImU32 color, float radius)
{
    m_points.Add(Vertex(p, color, radius));
}

void Viewport3D::Label(glm::dvec3 p, const char* text, ImU32 color) const
{
    // A dark shadow keeps the text readable over the surface.
    if (const std::optional<ImVec2> s = ToScreen(p))
        DrawLabel(m_canvas.drawList, *s, text, color, Theme::kLabelShadowColor);
}

void Viewport3D::LabeledPoint(glm::dvec3 p, const char* text, ImU32 color, float radius)
{
    Point(p, color, radius);
    Label(p, text, color);
}

void Viewport3D::ApplyPendingFit()
{
    if (!m_pendingFit)
        return;

    m_camera.Fit(m_pendingFit->min, m_pendingFit->max, m_canvas.Aspect());
    m_pendingFit.reset();
}

void Viewport3D::HandleInput()
{
    const ImGuiIO& io = ImGui::GetIO();

    if (m_canvasActive)
    {
        const double dx = io.MouseDelta.x;
        const double dy = io.MouseDelta.y;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            if (!m_mouseClaimed)
                m_camera.Orbit(dx, dy);
        }
        else if (ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle))
        {
            m_camera.Pan(dx, dy, m_canvas.size.y);
        }
    }

    if (m_canvas.hovered && !m_wheelTaken && io.MouseWheel != 0.0f)
        m_camera.Zoom(io.MouseWheel);
}

bool Viewport3D::PreparePrograms()
{
    if (m_programsFailed)
        return false;
    if (m_meshProgram)
        return true;

    // Build logs the reason on failure; the flag keeps it from retrying every frame.
    m_meshProgram = ShaderProgram::Build("Viewport3D mesh", Shaders::kMeshVertex, Shaders::kMeshFragment);
    m_lineProgram = ShaderProgram::Build("Viewport3D lines", Shaders::kPrimitiveVertex, Shaders::kLineGeometry,
                                         Shaders::kLineFragment);
    m_pointProgram = ShaderProgram::Build("Viewport3D points", Shaders::kPrimitiveVertex, Shaders::kPointGeometry,
                                          Shaders::kPointFragment);
    if (!m_meshProgram || !m_lineProgram || !m_pointProgram)
    {
        m_meshProgram = {};
        m_programsFailed = true;
        return false;
    }
    return true;
}

void Viewport3D::SetPrimitiveUniforms(const ShaderProgram& program, float lift) const
{
    program.Set("uView", glm::mat4(m_view));
    program.Set("uProjection", glm::mat4(m_projection));
    program.Set("uViewportSize", glm::vec2(static_cast<float>(m_target.Width()), static_cast<float>(m_target.Height())));
    program.Set("uLift", lift);
}

void Viewport3D::DrawPrimitives()
{
    if ((m_lines.IsEmpty() && m_points.IsEmpty()) || !PreparePrograms())
        return;

    // Blend color, but keep the framebuffer's alpha at 1 so the image stays opaque in ImGui.
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    m_lineProgram.Use();
    SetPrimitiveUniforms(m_lineProgram, kLineLift);
    m_lines.Draw(GL_LINES);

    m_pointProgram.Use();
    SetPrimitiveUniforms(m_pointProgram, kPointLift);
    m_points.Draw(GL_POINTS);

    glUseProgram(0);
    glDisable(GL_BLEND);
}

void Viewport3D::DrawGrid()
{
    // A power-of-ten step that keeps roughly kGridLinesPerDistance lines across the view.
    const double step = Detail::PowerOfTenAtLeast(m_camera.Distance() / kGridLinesPerDistance);
    const glm::dvec3 target = m_camera.Target();
    const auto cx = static_cast<long long>(std::round(target.x / step));
    const auto cy = static_cast<long long>(std::round(target.y / step));

    const double x0 = static_cast<double>(cx - kGridHalfLines) * step;
    const double x1 = static_cast<double>(cx + kGridHalfLines) * step;
    const double y0 = static_cast<double>(cy - kGridHalfLines) * step;
    const double y1 = static_cast<double>(cy + kGridHalfLines) * step;

    // The x axis is the line y = 0 and the y axis the line x = 0.
    for (long long i = cx - kGridHalfLines; i <= cx + kGridHalfLines; ++i)
    {
        const double x = static_cast<double>(i) * step;
        Line({x, y0, 0.0}, {x, y1, 0.0}, i == 0 ? Theme::kYAxisColor : Theme::kGridColor);
    }
    for (long long j = cy - kGridHalfLines; j <= cy + kGridHalfLines; ++j)
    {
        const double y = static_cast<double>(j) * step;
        Line({x0, y, 0.0}, {x1, y, 0.0}, j == 0 ? Theme::kXAxisColor : Theme::kGridColor);
    }

    // Each axis is labeled at the grid's positive edge if it crosses the grid.
    if (y0 <= 0.0 && 0.0 <= y1)
        Label({x1, 0.0, 0.0}, "x", Theme::kXAxisColor);
    if (x0 <= 0.0 && 0.0 <= x1)
        Label({0.0, y1, 0.0}, "y", Theme::kYAxisColor);

    const double zLength = 0.5 * kGridHalfLines * step;
    Line({0.0, 0.0, 0.0}, {0.0, 0.0, zLength}, Theme::kZAxisColor, 1.5f);
    Label({0.0, 0.0, zLength}, "z", Theme::kZAxisColor);
}

void Viewport3D::DrawGnomon() const
{
    static constexpr std::array<ImU32, 3> kColors = {Theme::kXAxisColor, Theme::kYAxisColor, Theme::kZAxisColor};
    static constexpr std::array<const char*, 3> kNames = {"x", "y", "z"};

    const ImVec2 origin(m_canvas.min.x + kGnomonMargin + kGnomonLength,
                        m_canvas.Max().y - kGnomonMargin - kGnomonLength);
    m_canvas.drawList->AddCircleFilled(origin, kGnomonLength + 0.5f * kGnomonMargin, kGnomonBackdropColor);

    // Back to front, so the axes nearer the eye are drawn over the others.
    for (const Detail::GnomonAxis& a : Detail::GnomonAxes(m_view))
    {
        const auto i = static_cast<std::size_t>(a.axis);
        const ImVec2 tip(origin.x + static_cast<float>(a.dir.x) * kGnomonLength,
                         origin.y + static_cast<float>(a.dir.y) * kGnomonLength);
        m_canvas.drawList->AddLine(origin, tip, kColors[i], kGnomonThickness);
        DrawLabel(m_canvas.drawList, tip, kNames[i], kColors[i], Theme::kLabelShadowColor);
    }
}

} // namespace Graphics
