#include "Viewport3D.hpp"

#include "Viewport3DShaders.hpp"

#include <glad/gl.h>
#include <glm/mat3x3.hpp>
#include <glm/matrix.hpp>
#include <glm/vec2.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>

namespace NURBS::Viewer
{

namespace
{

constexpr float kMinCanvasPixels = 1.0f;
constexpr int kGridHalfLines = 10;
constexpr double kGridLinesPerDistance = 8.0;
constexpr int kMaxSamples = 4;

// The same background and grid colors as Viewport2D.
constexpr float kBackground[4] = {24.0f / 255.0f, 24.0f / 255.0f, 28.0f / 255.0f, 1.0f};
constexpr ImU32 kGridColor = IM_COL32(48, 48, 56, 255);
constexpr ImU32 kXAxisColor = IM_COL32(200, 80, 80, 255);
constexpr ImU32 kYAxisColor = IM_COL32(100, 190, 90, 255);
constexpr ImU32 kZAxisColor = IM_COL32(90, 140, 240, 255);
constexpr ImU32 kLabelShadowColor = IM_COL32(0, 0, 0, 200);

// How far lines and points are lifted toward the eye, in multiples of their
// size (a line's thickness, a point's radius). A full thickness keeps lines on
// a surface seen nearly edge-on from breaking up.
constexpr float kLineLift = 1.0f;
constexpr float kPointLift = 1.0f;

// Signed distance-like value of a clip-space point from the near plane
// (z = -w in GL clip space); non-negative means in front of it.
[[nodiscard]] double NearPlaneSide(glm::dvec4 clip)
{
    return clip.z + clip.w;
}

// Logs a framebuffer that isn't complete. Returns true if it is complete.
[[nodiscard]] bool CheckFramebuffer(GLuint framebuffer, const char* name)
{
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (status == GL_FRAMEBUFFER_COMPLETE)
        return true;

    std::fprintf(stderr, "Viewport3D: %s framebuffer incomplete (status 0x%x)\n", name, status);
    return false;
}

} // namespace

void Viewport3D::Begin(const char* id)
{
    m_drawList = ImGui::GetWindowDrawList();
    m_min = ImGui::GetCursorScreenPos();
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    m_size = ImVec2(std::max(avail.x, kMinCanvasPixels), std::max(avail.y, kMinCanvasPixels));

    ImGui::InvisibleButton(id, m_size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    m_hovered = ImGui::IsItemHovered();
    // Claim the wheel so zooming doesn't also scroll an enclosing window.
    if (m_hovered)
        ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);

    ApplyPendingFit();
    HandleInput();

    const double aspect = static_cast<double>(m_size.x) / static_cast<double>(m_size.y);
    m_view = m_camera.View();
    m_projection = m_camera.Projection(aspect);
    m_viewProjection = m_projection * m_view;

    // The framebuffer is in physical pixels, which differ from ImGui's on high-DPI displays.
    const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
    m_pixelScale = scale.y;
    const int width = std::max(1, static_cast<int>(std::lround(m_size.x * scale.x)));
    const int height = std::max(1, static_cast<int>(std::lround(m_size.y * scale.y)));
    m_framebufferBound = PrepareFramebuffers(width, height);
    if (m_framebufferBound)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, m_msaaFramebuffer.Get());
        glViewport(0, 0, width, height);
        glEnable(GL_DEPTH_TEST);
        glClearColor(kBackground[0], kBackground[1], kBackground[2], kBackground[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    const ImVec2 max(m_min.x + m_size.x, m_min.y + m_size.y);
    m_drawList->PushClipRect(m_min, max, true);
    if (m_framebufferBound)
    {
        // Added now so labels land on top; ImGui samples the texture when it
        // renders, after End has resolved into it. GL rows run bottom-up, so flip v.
        const auto texture = static_cast<ImTextureID>(m_colorTexture.Get());
        m_drawList->AddImage(texture, m_min, max, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
    }

    m_lines.Clear();
    m_points.Clear();
    DrawGrid();
}

void Viewport3D::End()
{
    assert(m_drawList != nullptr);
    if (m_framebufferBound)
    {
        DrawPrimitives();

        glBindFramebuffer(GL_READ_FRAMEBUFFER, m_msaaFramebuffer.Get());
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_framebuffer.Get());
        glBlitFramebuffer(0, 0, m_framebufferWidth, m_framebufferHeight, 0, 0, m_framebufferWidth, m_framebufferHeight,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);

        // App::Run sets the viewport again before ImGui renders.
        glDisable(GL_DEPTH_TEST);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        m_framebufferBound = false;
    }
    m_lines.Clear();
    m_points.Clear();
    m_drawList->PopClipRect();
    m_drawList = nullptr;
}

void Viewport3D::DrawMesh(const TriangleMesh& mesh, glm::vec3 frontColor, glm::vec3 backColor)
{
    if (!m_framebufferBound || !PreparePrograms() || mesh.IsEmpty())
        return;

    m_meshProgram.Use();
    m_meshProgram.Set("uView", glm::mat4(m_view));
    m_meshProgram.Set("uProjection", glm::mat4(m_projection));
    m_meshProgram.Set("uNormalMatrix", glm::mat3(glm::transpose(glm::inverse(m_view))));
    m_meshProgram.Set("uFrontColor", frontColor);
    m_meshProgram.Set("uBackColor", backColor);
    mesh.Draw();
    glUseProgram(0);
}

void Viewport3D::FitTo(glm::dvec3 min, glm::dvec3 max)
{
    m_pendingFit = Box{min, max};
}

glm::dvec4 Viewport3D::ToClip(glm::dvec3 p) const
{
    return m_viewProjection * glm::dvec4(p, 1.0);
}

ImVec2 Viewport3D::ClipToScreen(glm::dvec4 clip) const
{
    const double x = clip.x / clip.w;
    const double y = clip.y / clip.w;
    return {static_cast<float>(m_min.x + 0.5 * (x + 1.0) * m_size.x),
            static_cast<float>(m_min.y + 0.5 * (1.0 - y) * m_size.y)};
}

std::optional<ImVec2> Viewport3D::ToScreen(glm::dvec3 p) const
{
    const glm::dvec4 clip = ToClip(p);
    if (NearPlaneSide(clip) < 0.0 || clip.w <= 0.0)
        return std::nullopt;
    return ClipToScreen(clip);
}

PrimitiveVertex Viewport3D::Vertex(glm::dvec3 p, ImU32 color, float size) const
{
    const ImVec4 c = ImGui::ColorConvertU32ToFloat4(color);
    return {glm::vec3(p), glm::vec4(c.x, c.y, c.z, c.w), size * m_pixelScale};
}

void Viewport3D::Polyline(std::span<const glm::dvec3> points, ImU32 color, float thickness)
{
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        m_lines.Add(Vertex(points[i - 1], color, thickness));
        m_lines.Add(Vertex(points[i], color, thickness));
    }
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
    if (const std::optional<ImVec2> s = ToScreen(p))
    {
        // A dark shadow keeps the text readable over the surface.
        const ImVec2 at(s->x + 6.0f, s->y - ImGui::GetFontSize() - 2.0f);
        m_drawList->AddText(ImVec2(at.x + 1.0f, at.y + 1.0f), kLabelShadowColor, text);
        m_drawList->AddText(at, color, text);
    }
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

    m_camera.Fit(m_pendingFit->min, m_pendingFit->max, static_cast<double>(m_size.x) / static_cast<double>(m_size.y));
    m_pendingFit.reset();
}

void Viewport3D::HandleInput()
{
    const ImGuiIO& io = ImGui::GetIO();

    if (ImGui::IsItemActive())
    {
        const double dx = io.MouseDelta.x;
        const double dy = io.MouseDelta.y;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
            m_camera.Orbit(dx, dy);
        else if (ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle))
            m_camera.Pan(dx, dy, m_size.y);
    }

    if (m_hovered && io.MouseWheel != 0.0f)
        m_camera.Zoom(io.MouseWheel);
}

bool Viewport3D::PrepareFramebuffers(int width, int height)
{
    if (m_framebufferFailed)
        return false;

    if (!m_framebuffer)
    {
        GLuint id = 0;
        glGenFramebuffers(1, &id);
        m_msaaFramebuffer.Reset(id);
        glGenRenderbuffers(1, &id);
        m_msaaColor.Reset(id);
        glGenRenderbuffers(1, &id);
        m_msaaDepth.Reset(id);
        glGenFramebuffers(1, &id);
        m_framebuffer.Reset(id);
        glGenTextures(1, &id);
        m_colorTexture.Reset(id);

        GLint maxSamples = 0;
        glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
        m_samples = std::clamp(maxSamples, 1, kMaxSamples);
    }

    if (width == m_framebufferWidth && height == m_framebufferHeight)
        return true;

    // Same objects, new storage.
    glBindRenderbuffer(GL_RENDERBUFFER, m_msaaColor.Get());
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, m_samples, GL_RGBA8, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, m_msaaDepth.Get());
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, m_samples, GL_DEPTH_COMPONENT24, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, m_msaaFramebuffer.Get());
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, m_msaaColor.Get());
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_msaaDepth.Get());

    glBindTexture(GL_TEXTURE_2D, m_colorTexture.Get());
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer.Get());
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colorTexture.Get(), 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Logged once; labels still draw without the GL image.
    if (!CheckFramebuffer(m_msaaFramebuffer.Get(), "multisampled") || !CheckFramebuffer(m_framebuffer.Get(), "resolve"))
    {
        m_framebufferFailed = true;
        return false;
    }

    m_framebufferWidth = width;
    m_framebufferHeight = height;
    return true;
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
    program.Set("uViewportSize",
                glm::vec2(static_cast<float>(m_framebufferWidth), static_cast<float>(m_framebufferHeight)));
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
    const double step = std::pow(10.0, std::ceil(std::log10(m_camera.Distance() / kGridLinesPerDistance)));
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
        Line({x, y0, 0.0}, {x, y1, 0.0}, i == 0 ? kYAxisColor : kGridColor);
    }
    for (long long j = cy - kGridHalfLines; j <= cy + kGridHalfLines; ++j)
    {
        const double y = static_cast<double>(j) * step;
        Line({x0, y, 0.0}, {x1, y, 0.0}, j == 0 ? kXAxisColor : kGridColor);
    }

    // Each axis is labeled at the grid's positive edge if it crosses the grid.
    if (y0 <= 0.0 && 0.0 <= y1)
        Label({x1, 0.0, 0.0}, "x", kXAxisColor);
    if (x0 <= 0.0 && 0.0 <= x1)
        Label({0.0, y1, 0.0}, "y", kYAxisColor);

    const double zLength = 0.5 * kGridHalfLines * step;
    Line({0.0, 0.0, 0.0}, {0.0, 0.0, zLength}, kZAxisColor, 1.5f);
    Label({0.0, 0.0, zLength}, "z", kZAxisColor);
}

} // namespace NURBS::Viewer
