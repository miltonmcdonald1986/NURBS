#include <Graphics/App.hpp>

#include "BSplineBasisScene.hpp"
#include "BernsteinScene.hpp"
#include "BezierScene.hpp"
#include "BezierSurfaceScene.hpp"
#include "HornerScene.hpp"
#include "PowerBasisSurfaceScene.hpp"
#include "RationalBezierScene.hpp"
#include "RationalBezierSurfaceScene.hpp"

#include <cstdlib>
#include <memory>
#include <vector>

// An escaping exception (e.g. bad_alloc) terminating the viewer is acceptable.
int main() // NOLINT(bugprone-exception-escape)
{
    Graphics::App app(1280, 720, "NURBS Viewer");
    if (!app.IsValid())
        return EXIT_FAILURE;

    // Declared after app so the scenes, and any GL objects they own, are
    // destroyed while the GL context still exists.
    std::vector<std::unique_ptr<Graphics::Scene>> scenes;
    scenes.push_back(std::make_unique<NURBS::Viewer::HornerScene>());
    scenes.push_back(std::make_unique<NURBS::Viewer::PowerBasisSurfaceScene>());
    scenes.push_back(std::make_unique<NURBS::Viewer::BernsteinScene>());
    scenes.push_back(std::make_unique<NURBS::Viewer::BezierScene>());
    scenes.push_back(std::make_unique<NURBS::Viewer::RationalBezierScene>());
    scenes.push_back(std::make_unique<NURBS::Viewer::BezierSurfaceScene>());
    scenes.push_back(std::make_unique<NURBS::Viewer::RationalBezierSurfaceScene>());
    scenes.push_back(std::make_unique<NURBS::Viewer::BSplineBasisScene>());

    app.Run(scenes);
    return EXIT_SUCCESS;
}
