#include <gtest/gtest.h>

#include <Graphics/Canvas.hpp>

#include <glm/glm.hpp>
#include <imgui.h>

TEST(ToVec4, Extremes)
{
    EXPECT_EQ(Graphics::ToVec4(IM_COL32(255, 0, 0, 255)), glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    EXPECT_EQ(Graphics::ToVec4(IM_COL32(0, 255, 0, 0)), glm::vec4(0.0f, 1.0f, 0.0f, 0.0f));
    EXPECT_EQ(Graphics::ToVec4(IM_COL32(0, 0, 255, 255)), glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
}

TEST(ToVec4, MidValues)
{
    const glm::vec4 color = Graphics::ToVec4(IM_COL32(128, 64, 32, 200));
    EXPECT_FLOAT_EQ(color.r, 128.0f / 255.0f);
    EXPECT_FLOAT_EQ(color.g, 64.0f / 255.0f);
    EXPECT_FLOAT_EQ(color.b, 32.0f / 255.0f);
    EXPECT_FLOAT_EQ(color.a, 200.0f / 255.0f);
}
