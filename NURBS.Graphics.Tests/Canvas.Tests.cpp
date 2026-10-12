#include <gtest/gtest.h>

#include <Graphics/Canvas.hpp>

#include <glm/glm.hpp>
#include <imgui.h>

namespace
{

// An ImGui context with no platform or renderer backend. Claiming texture
// support lets the font atlas build itself on the first frame; nothing ever
// uploads it, which the tests don't need.
class ImGuiFrameTest : public testing::Test
{
protected:
    static constexpr ImVec2 kWindowPos{0.0f, 0.0f};
    static constexpr ImVec2 kWindowSize{400.0f, 300.0f};

    void SetUp() override
    {
        m_context = ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.IniFilename = nullptr;
        io.DisplaySize = ImVec2(800.0f, 600.0f);
        io.DeltaTime = 1.0f / 60.0f;
        io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    }

    void TearDown() override { ImGui::DestroyContext(m_context); }

    // Runs body inside a fixed, undecorated window for one frame.
    template <typename Body>
    void Frame(Body&& body)
    {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(kWindowPos);
        ImGui::SetNextWindowSize(kWindowSize);
        ImGui::Begin("Test", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);
        body();
        ImGui::End();
        ImGui::Render();
    }

private:
    ImGuiContext* m_context = nullptr;
};

} // namespace

TEST(Canvas, MaxAndAspect)
{
    Graphics::Canvas canvas;
    canvas.min = ImVec2(10.0f, 20.0f);
    canvas.size = ImVec2(200.0f, 100.0f);

    const ImVec2 max = canvas.Max();
    EXPECT_EQ(max.x, 210.0f);
    EXPECT_EQ(max.y, 120.0f);
    EXPECT_DOUBLE_EQ(canvas.Aspect(), 2.0);
}

using BeginCanvas = ImGuiFrameTest;

TEST_F(BeginCanvas, ClaimsRemainingContentRegion)
{
    Frame([] {
        const ImVec2 cursor = ImGui::GetCursorScreenPos();
        const ImVec2 avail = ImGui::GetContentRegionAvail();

        const Graphics::Canvas canvas = Graphics::BeginCanvas("canvas");
        EXPECT_EQ(canvas.drawList, ImGui::GetWindowDrawList());
        EXPECT_EQ(canvas.min.x, cursor.x);
        EXPECT_EQ(canvas.min.y, cursor.y);
        EXPECT_EQ(canvas.size.x, avail.x);
        EXPECT_EQ(canvas.size.y, avail.y);

        // The canvas is the last item, as ImGui::IsItemActive relies on.
        EXPECT_EQ(ImGui::GetItemRectMin().x, canvas.min.x);
        EXPECT_EQ(ImGui::GetItemRectMin().y, canvas.min.y);
        EXPECT_EQ(ImGui::GetItemRectSize().x, canvas.size.x);
        EXPECT_EQ(ImGui::GetItemRectSize().y, canvas.size.y);
    });
}

TEST_F(BeginCanvas, AtLeastOnePixel)
{
    Frame([] {
        // Use up the window's height, leaving no room below.
        ImGui::Dummy(ImGui::GetContentRegionAvail());
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        ASSERT_LT(avail.y, 1.0f);

        const Graphics::Canvas canvas = Graphics::BeginCanvas("canvas");
        EXPECT_EQ(canvas.size.x, avail.x);
        EXPECT_EQ(canvas.size.y, 1.0f);
    });
}

TEST_F(BeginCanvas, HoveredFollowsMouse)
{
    // Hovering is resolved against the previous frame's windows, so each
    // check runs a frame to settle first.
    const auto hoveredAt = [this](ImVec2 mouse) {
        ImGui::GetIO().MousePos = mouse;
        bool hovered = false;
        for (int frame = 0; frame < 2; ++frame)
        {
            Frame([&hovered] { hovered = Graphics::BeginCanvas("canvas").hovered; });
        }
        return hovered;
    };

    EXPECT_TRUE(hoveredAt(ImVec2(200.0f, 150.0f)));
    EXPECT_FALSE(hoveredAt(ImVec2(600.0f, 450.0f)));
}

using DrawLabel = ImGuiFrameTest;

TEST_F(DrawLabel, Text)
{
    Frame([] {
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const int before = drawList->VtxBuffer.Size;
        const ImVec2 anchor(100.0f, 100.0f);

        Graphics::DrawLabel(drawList, anchor, "A", IM_COL32(255, 0, 0, 255));

        ASSERT_GT(drawList->VtxBuffer.Size, before);
        for (int i = before; i < drawList->VtxBuffer.Size; ++i)
        {
            const ImDrawVert& v = drawList->VtxBuffer[i];
            EXPECT_EQ(v.col, IM_COL32(255, 0, 0, 255));
            // Right of and above the anchor.
            EXPECT_GE(v.pos.x, anchor.x);
            EXPECT_LT(v.pos.y, anchor.y);
        }
    });
}

TEST_F(DrawLabel, ShadowSitsBehindAndOffset)
{
    Frame([] {
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const int before = drawList->VtxBuffer.Size;

        Graphics::DrawLabel(drawList, ImVec2(100.0f, 100.0f), "A", IM_COL32(255, 0, 0, 255), IM_COL32(0, 0, 0, 255));

        // The shadow comes first so the text draws over it, one pixel down
        // and right of the text.
        const int count = drawList->VtxBuffer.Size - before;
        ASSERT_GT(count, 0);
        ASSERT_EQ(count % 2, 0);
        const int half = count / 2;
        for (int i = 0; i < half; ++i)
        {
            const ImDrawVert& shadow = drawList->VtxBuffer[before + i];
            const ImDrawVert& text = drawList->VtxBuffer[before + half + i];
            EXPECT_EQ(shadow.col, IM_COL32(0, 0, 0, 255));
            EXPECT_EQ(text.col, IM_COL32(255, 0, 0, 255));
            EXPECT_EQ(shadow.pos.x, text.pos.x + 1.0f);
            EXPECT_EQ(shadow.pos.y, text.pos.y + 1.0f);
        }
    });
}

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
