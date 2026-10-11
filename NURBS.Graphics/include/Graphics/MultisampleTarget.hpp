/// @file
/// MultisampleTarget, an offscreen multisampled render target.

#pragma once

#include "GlHandle.hpp"

namespace Graphics
{

/// An offscreen render target: a multisampled framebuffer (RGBA8 color and
/// 24-bit depth) to draw into, and a single-sample color texture it resolves
/// into for display.
///
/// GL objects are created on the first Prepare, so the GL context must be
/// current then, and still current when the target is destroyed.
class MultisampleTarget
{
public:
    /// Creates the objects on first use and sizes them to @p width x @p height pixels.
    /// @param width The width, in pixels.
    /// @param height The height, in pixels.
    /// @return False if either framebuffer is incomplete; that is logged once,
    ///         and every later call returns false too.
    bool Prepare(int width, int height);

    /// Binds the multisampled framebuffer for drawing and sets the viewport to it.
    void Bind() const;
    /// Resolves the multisampled color into ColorTexture and unbinds.
    void Resolve() const;

    /// The texture that Resolve writes into.
    /// @return The texture's name.
    [[nodiscard]] GLuint ColorTexture() const { return m_colorTexture.Get(); }
    /// The width from the last Prepare.
    /// @return The width, in pixels.
    [[nodiscard]] int Width() const { return m_width; }
    /// The height from the last Prepare.
    /// @return The height, in pixels.
    [[nodiscard]] int Height() const { return m_height; }

private:
    void Create();
    void Allocate(int width, int height);

    GlFramebuffer m_msaaFramebuffer;
    GlRenderbuffer m_msaaColor;
    GlRenderbuffer m_msaaDepth;
    GlFramebuffer m_resolveFramebuffer;
    GlTexture m_colorTexture;
    int m_samples = 0;
    int m_width = 0;
    int m_height = 0;
    bool m_failed = false;
};

} // namespace Graphics
