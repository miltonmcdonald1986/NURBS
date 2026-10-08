#pragma once

namespace NURBS::Viewer
{

// One self-contained visualization (e.g. an algorithm from the library). The
// App owns the window and frame loop; a scene owns its own state and only
// draws into the regions the App hands it.
class Scene
{
public:
    Scene() = default;
    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;
    Scene(Scene&&) = delete;
    Scene& operator=(Scene&&) = delete;
    virtual ~Scene() = default;

    [[nodiscard]] virtual const char* Name() const = 0;

    // Controls, drawn inside the App's side panel.
    virtual void DrawUI() = 0;

    // Drawn inside the App's main viewport window, which has no padding.
    virtual void DrawViewport() = 0;
};

} // namespace NURBS::Viewer
