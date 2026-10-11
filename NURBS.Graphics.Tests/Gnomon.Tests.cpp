#include <gtest/gtest.h>

#include "Gnomon.hpp"

#include <Graphics/OrbitCamera.hpp>

#include <glm/glm.hpp>

#include <array>

namespace
{

using Graphics::Detail::GnomonAxes;
using Graphics::Detail::GnomonAxis;

// The entry for axis i (0 = x, 1 = y, 2 = z).
const GnomonAxis& Find(const std::array<GnomonAxis, 3>& axes, int i)
{
    for (const GnomonAxis& a : axes)
    {
        if (a.axis == i)
            return a;
    }
    ADD_FAILURE() << "axis " << i << " missing";
    return axes[0];
}

void ExpectBackToFront(const std::array<GnomonAxis, 3>& axes)
{
    EXPECT_LE(axes[0].depth, axes[1].depth);
    EXPECT_LE(axes[1].depth, axes[2].depth);
}

} // namespace

TEST(GnomonAxes, IdentityView)
{
    const auto axes = GnomonAxes(glm::dmat4(1.0));
    ExpectBackToFront(axes);

    const GnomonAxis& x = Find(axes, 0);
    EXPECT_DOUBLE_EQ(x.dir.x, 1.0);
    EXPECT_DOUBLE_EQ(x.dir.y, 0.0);
    // Screen y runs down, so view-space +y points up the screen.
    const GnomonAxis& y = Find(axes, 1);
    EXPECT_DOUBLE_EQ(y.dir.x, 0.0);
    EXPECT_DOUBLE_EQ(y.dir.y, -1.0);
    // z points at the eye: drawn last, with no length on screen.
    EXPECT_EQ(axes[2].axis, 2);
    EXPECT_DOUBLE_EQ(axes[2].depth, 1.0);
    EXPECT_DOUBLE_EQ(glm::length(axes[2].dir), 0.0);
}

TEST(GnomonAxes, OrbitCameraKeepsZUp)
{
    const Graphics::OrbitCamera camera;
    const auto axes = GnomonAxes(camera.View());
    ExpectBackToFront(axes);

    for (int i = 0; i < 3; ++i)
        EXPECT_LE(glm::length(Find(axes, i).dir), 1.0 + 1e-12);

    const GnomonAxis& z = Find(axes, 2);
    EXPECT_NEAR(z.dir.x, 0.0, 1e-12);
    EXPECT_LT(z.dir.y, 0.0);
}

TEST(GnomonAxes, IgnoresTranslation)
{
    const Graphics::OrbitCamera camera;
    Graphics::OrbitCamera panned = camera;
    panned.Pan(120.0, -45.0, 600.0);
    panned.Zoom(3.0);
    ASSERT_NE(camera.Target(), panned.Target());

    const auto a = GnomonAxes(camera.View());
    const auto b = GnomonAxes(panned.View());
    for (std::size_t i = 0; i < a.size(); ++i)
    {
        EXPECT_EQ(a[i].axis, b[i].axis);
        EXPECT_NEAR(a[i].dir.x, b[i].dir.x, 1e-12);
        EXPECT_NEAR(a[i].dir.y, b[i].dir.y, 1e-12);
        EXPECT_NEAR(a[i].depth, b[i].depth, 1e-12);
    }
}
