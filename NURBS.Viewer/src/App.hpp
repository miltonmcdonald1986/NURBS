#pragma once

#include "Scene.hpp"

#include <cstddef>
#include <memory>
#include <span>

struct GLFWwindow;

namespace NURBS::Viewer
{

// Owns the GLFW window and the ImGui context, and runs the frame loop. Each
// frame it lays out a side panel (scene picker + the active scene's controls)
// and a main viewport (the active scene's drawing). It knows nothing about
// NURBS; everything specific lives in the scenes.
class App
{
public:
    App(int width, int height, const char* title);
    App(const App&) = delete;
    App& operator=(const App&) = delete;
    App(App&&) = delete;
    App& operator=(App&&) = delete;
    ~App();

    // False if the window or GL context could not be created.
    [[nodiscard]] bool IsValid() const { return m_window != nullptr; }

    // Runs until the window is closed. Precondition: IsValid() and scenes is non-empty.
    void Run(std::span<const std::unique_ptr<Scene>> scenes);

private:
    void DrawFrame(std::span<const std::unique_ptr<Scene>> scenes);

    GLFWwindow* m_window = nullptr;
    bool m_glfwInitialized = false;
    std::size_t m_activeScene = 0;
};

} // namespace NURBS::Viewer
