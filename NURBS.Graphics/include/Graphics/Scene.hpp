/// @file
/// The Scene interface that App runs.

#pragma once

/// An app shell, viewports, meshes, and widgets for interactive OpenGL and
/// Dear ImGui visualizations.
///
/// NURBS.Graphics is built on GLFW, OpenGL 3.3, and Dear ImGui, and has no
/// dependency on NURBS.Lib. App runs a list of Scene objects; Viewport2D and
/// Viewport3D draw into a Canvas; and the GL wrappers manage meshes, lines,
/// and points.
namespace Graphics
{

/// One self-contained visualization, picked from the App's scene list. The
/// App owns the window and frame loop; a scene owns its own state and only
/// draws into the regions the App hands it.
class Scene
{
public:
    Scene() = default;
    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;
    Scene(Scene&&) = delete;
    Scene& operator=(Scene&&) = delete;
    virtual ~Scene() = default;

    /// The name shown in the App's scene picker.
    /// @return A null-terminated string that outlives the scene's use by the App.
    [[nodiscard]] virtual const char* Name() const = 0;

    /// Draws the controls, inside the App's side panel.
    virtual void DrawUI() = 0;

    /// Draws the scene, inside the App's main viewport window, which has no padding.
    virtual void DrawViewport() = 0;

    /// Draws the graphics-only options, such as render quality, projection,
    /// and navigation, inside the App's "View" overlay on the viewport. Kept
    /// apart from DrawUI, which holds what the scene is about.
    virtual void DrawViewOptions() {}
};

} // namespace Graphics
