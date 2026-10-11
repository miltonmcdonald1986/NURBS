#include <Graphics/MultisampleTarget.hpp>

#include <algorithm>
#include <cstdio>

namespace Graphics
{

namespace
{

constexpr int kMaxSamples = 4;

// Logs a framebuffer that isn't complete. Returns true if it is complete.
[[nodiscard]] bool CheckFramebuffer(GLuint framebuffer, const char* name)
{
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (status == GL_FRAMEBUFFER_COMPLETE)
        return true;

    std::fprintf(stderr, "MultisampleTarget: %s framebuffer incomplete (status 0x%x)\n", name, status);
    return false;
}

} // namespace

bool MultisampleTarget::Prepare(int width, int height)
{
    if (m_failed)
        return false;
    if (!m_resolveFramebuffer)
        Create();
    if (width == m_width && height == m_height)
        return true;

    Allocate(width, height);
    if (!CheckFramebuffer(m_msaaFramebuffer.Get(), "multisampled") ||
        !CheckFramebuffer(m_resolveFramebuffer.Get(), "resolve"))
    {
        m_failed = true;
        return false;
    }

    m_width = width;
    m_height = height;
    return true;
}

void MultisampleTarget::Bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_msaaFramebuffer.Get());
    glViewport(0, 0, m_width, m_height);
}

void MultisampleTarget::Resolve() const
{
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_msaaFramebuffer.Get());
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_resolveFramebuffer.Get());
    glBlitFramebuffer(0, 0, m_width, m_height, 0, 0, m_width, m_height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MultisampleTarget::Create()
{
    m_msaaFramebuffer = GlFramebuffer::Create();
    m_msaaColor = GlRenderbuffer::Create();
    m_msaaDepth = GlRenderbuffer::Create();
    m_resolveFramebuffer = GlFramebuffer::Create();
    m_colorTexture = GlTexture::Create();

    GLint maxSamples = 0;
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
    m_samples = std::clamp(maxSamples, 1, kMaxSamples);
}

void MultisampleTarget::Allocate(int width, int height)
{
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

    glBindFramebuffer(GL_FRAMEBUFFER, m_resolveFramebuffer.Get());
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colorTexture.Get(), 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace Graphics
