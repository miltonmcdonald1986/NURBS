add_executable(NURBS.Viewer)

target_sources(NURBS.Viewer
    PRIVATE
        src/BernsteinScene.cpp
        src/BernsteinScene.hpp
        src/BezierScene.cpp
        src/BezierScene.hpp
        src/BezierSurfaceScene.cpp
        src/BezierSurfaceScene.hpp
        src/BSplineBasisScene.cpp
        src/BSplineBasisScene.hpp
        src/ControlNet.cpp
        src/ControlNet.hpp
        src/ControlPolygon.cpp
        src/ControlPolygon.hpp
        src/HornerScene.cpp
        src/HornerScene.hpp
        src/PowerBasisSurfaceScene.cpp
        src/PowerBasisSurfaceScene.hpp
        src/RationalBezierScene.cpp
        src/RationalBezierScene.hpp
        src/RationalBezierSurfaceScene.cpp
        src/RationalBezierSurfaceScene.hpp
        src/SceneWidgets.hpp
        src/SurfaceView.cpp
        src/SurfaceView.hpp
        src/Palette.hpp
        src/main.cpp
)

target_link_libraries(NURBS.Viewer
    PRIVATE
        NURBS::Lib
        Graphics::Graphics
)

set_target_properties(NURBS.Viewer PROPERTIES CXX_EXTENSIONS OFF)

if(EMSCRIPTEN)
    # Emits NURBS.Viewer.html with its .js and .wasm. The GLFW port and WebGL2
    # flags come from NURBS.Graphics. web/shell.html replaces Emscripten's
    # default page with a full-window canvas.
    set(shell ${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html)
    set_target_properties(NURBS.Viewer PROPERTIES SUFFIX ".html" LINK_DEPENDS ${shell})
    target_link_options(NURBS.Viewer PRIVATE -sALLOW_MEMORY_GROWTH=1 "SHELL:--shell-file ${shell}")
endif()
