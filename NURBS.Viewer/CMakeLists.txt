include(FetchContent)

FetchContent_Declare(glfw
    URL https://github.com/glfw/glfw/archive/refs/tags/3.5.1.tar.gz
    URL_HASH SHA256=5234f4f29473e9a06bc7847d8371858dd135d38466eeeaa652fdc9f8f9ff0c20
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM
)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# Dear ImGui ships no CMake build, so FetchContent only downloads it and the
# library target is defined here.
FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.92.9b.tar.gz
    URL_HASH SHA256=21d8a0a565e85dce943e375db00812c2f3f0ab21f3f0f7964e364a63422d7f99
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(imgui)

find_package(OpenGL REQUIRED)

add_library(imgui STATIC)

target_sources(imgui
    PRIVATE
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui SYSTEM
    PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui
    PUBLIC
        glfw
        OpenGL::GL
)

add_executable(NURBS.Viewer)

target_sources(NURBS.Viewer
    PRIVATE
        src/App.cpp
        src/App.hpp
        src/BernsteinScene.cpp
        src/BernsteinScene.hpp
        src/CurveSampling.hpp
        src/HornerScene.cpp
        src/HornerScene.hpp
        src/Scene.hpp
        src/Viewport2D.cpp
        src/Viewport2D.hpp
        src/main.cpp
)

target_link_libraries(NURBS.Viewer
    PRIVATE
        NURBS::Lib
        glm::glm
        imgui
        glfw
        OpenGL::GL
)

set_target_properties(NURBS.Viewer PROPERTIES CXX_EXTENSIONS OFF)
