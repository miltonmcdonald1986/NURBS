/// @file
/// The App class, which owns the window and runs the frame loop.

#pragma once

#include "Scene.hpp"

#include <cstddef>
#include <memory>
#include <span>

struct GLFWwindow;

namespace Graphics
{

/// Owns the GLFW window and the ImGui context, and runs the frame loop. Each
/// frame it lays out a side panel (scene picker + the active scene's controls)
/// and a main viewport (the active scene's drawing), with a collapsible "View"
/// overlay in its top-right corner (the scene's graphics-only options).
/// Everything specific to an application lives in its scenes.
class App
{
public:
    /// Creates the window, the GL context, and the ImGui context. Check
    /// IsValid() before calling Run().
    /// @param width The window width, in screen coordinates.
    /// @param height The window height, in screen coordinates.
    /// @param title The window title.
    App(int width, int height, const char* title);
    App(const App&) = delete;
    App& operator=(const App&) = delete;
    App(App&&) = delete;
    App& operator=(App&&) = delete;
    ~App();

    /// Whether the window and GL context were created.
    /// @return False if the window or GL context could not be created, or the
    ///         GL functions could not be loaded.
    [[nodiscard]] bool IsValid() const { return m_window != nullptr; }

    /// Runs until the window is closed.
    /// @param scenes The scenes to pick from. The first one starts active.
    /// @pre IsValid() and @p scenes is non-empty.
    void Run(std::span<const std::unique_ptr<Scene>> scenes);

private:
    void DrawFrame(std::span<const std::unique_ptr<Scene>> scenes);

    GLFWwindow* m_window = nullptr;
    bool m_glfwInitialized = false;
    std::size_t m_activeScene = 0;
};

} // namespace Graphics
