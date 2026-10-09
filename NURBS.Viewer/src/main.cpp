#include "App.hpp"
#include "BernsteinScene.hpp"
#include "HornerScene.hpp"

#include <cstdlib>
#include <memory>
#include <vector>

int main()
{
    NURBS::Viewer::App app(1280, 720, "NURBS Viewer");
    if (!app.IsValid())
        return EXIT_FAILURE;

    std::vector<std::unique_ptr<NURBS::Viewer::Scene>> scenes;
    scenes.push_back(std::make_unique<NURBS::Viewer::HornerScene>());
    scenes.push_back(std::make_unique<NURBS::Viewer::BernsteinScene>());

    app.Run(scenes);
    return EXIT_SUCCESS;
}
