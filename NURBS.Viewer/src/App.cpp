#include "App.hpp"

#include "ImGuiHelpers.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cassert>
#include <cstdio>

namespace NURBS::Viewer
{

namespace
{

constexpr float kSidePanelWidth = 380.0f;

void OnGlfwError(int error, const char* description)
{
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

} // namespace

App::App(int width, int height, const char* title)
{
    glfwSetErrorCallback(OnGlfwError);
    if (glfwInit() == GLFW_FALSE)
        return;
    m_glfwInitialized = true;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    m_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (m_window == nullptr)
        return;
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
}

App::~App()
{
    if (m_window != nullptr)
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(m_window);
    }
    if (m_glfwInitialized)
        glfwTerminate();
}

void App::Run(std::span<const std::unique_ptr<Scene>> scenes)
{
    assert(IsValid());
    assert(!scenes.empty());

    while (glfwWindowShouldClose(m_window) == GLFW_FALSE)
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        DrawFrame(scenes);

        ImGui::Render();
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(m_window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_window);
    }
}

void App::DrawFrame(std::span<const std::unique_ptr<Scene>> scenes)
{
    constexpr ImGuiWindowFlags kFixedWindow =
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const ImVec2 origin = viewport->WorkPos;
    const ImVec2 size = viewport->WorkSize;
    const float panelWidth = size.x < 2.0f * kSidePanelWidth ? size.x * 0.5f : kSidePanelWidth;

    Scene& active = *scenes[m_activeScene];

    ImGui::SetNextWindowPos(origin);
    ImGui::SetNextWindowSize(ImVec2(panelWidth, size.y));
    // Always reserve the scrollbar: scenes may size content to the panel width
    // (e.g. a square canvas), and an auto scrollbar would toggle on and off as
    // that content's height crosses the window height, wobbling the layout.
    if (ImGui::Begin("Controls", nullptr, kFixedWindow | ImGuiWindowFlags_AlwaysVerticalScrollbar))
    {
        NamedCombo("Scene", scenes, m_activeScene, [](const std::unique_ptr<Scene>& scene) { return scene->Name(); });
        ImGui::Separator();
        active.DrawUI();
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(origin.x + panelWidth, origin.y));
    ImGui::SetNextWindowSize(ImVec2(size.x - panelWidth, size.y));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    if (ImGui::Begin("Viewport", nullptr, kFixedWindow | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar |
                                              ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus))
    {
        active.DrawViewport();
    }
    ImGui::End();
    ImGui::PopStyleVar();
}

} // namespace NURBS::Viewer
