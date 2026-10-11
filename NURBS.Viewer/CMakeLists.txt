add_executable(NURBS.Viewer)

target_sources(NURBS.Viewer
    PRIVATE
        src/BernsteinScene.cpp
        src/BernsteinScene.hpp
        src/BezierScene.cpp
        src/BezierScene.hpp
        src/ControlPolygon.cpp
        src/ControlPolygon.hpp
        src/HornerScene.cpp
        src/HornerScene.hpp
        src/PowerBasisSurfaceScene.cpp
        src/PowerBasisSurfaceScene.hpp
        src/RationalBezierScene.cpp
        src/RationalBezierScene.hpp
        src/SceneWidgets.hpp
        src/main.cpp
)

target_link_libraries(NURBS.Viewer
    PRIVATE
        NURBS::Lib
        Graphics::Graphics
)

set_target_properties(NURBS.Viewer PROPERTIES CXX_EXTENSIONS OFF)
