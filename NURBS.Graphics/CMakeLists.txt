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

# glad is generated and vendored (see third_party/glad/README.md).
add_library(glad STATIC third_party/glad/gl.cpp)

target_include_directories(glad SYSTEM
    PUBLIC
        third_party/glad/include
)

# Rendering, cameras, viewports, and the app shell, with no dependency on NURBS.Lib.
add_library(NURBS.Graphics STATIC)
add_library(Graphics::Graphics ALIAS NURBS.Graphics)

target_sources(NURBS.Graphics
    PRIVATE
        src/App.cpp
        src/Canvas.cpp
        src/GlUtil.hpp
        src/Grid.hpp
        src/MultisampleTarget.cpp
        src/OrbitCamera.cpp
        src/Picking.cpp
        src/PrimitiveBatch.cpp
        src/ShaderProgram.cpp
        src/SurfaceMesh.cpp
        src/TriangleMesh.cpp
        src/Viewport2D.cpp
        src/Viewport3D.cpp
        src/Viewport3DShaders.hpp
        src/Widgets.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS include
        FILES
            include/Graphics/Animation.hpp
            include/Graphics/App.hpp
            include/Graphics/Bounds.hpp
            include/Graphics/Canvas.hpp
            include/Graphics/GlHandle.hpp
            include/Graphics/MultisampleTarget.hpp
            include/Graphics/OrbitCamera.hpp
            include/Graphics/Picking.hpp
            include/Graphics/PrimitiveBatch.hpp
            include/Graphics/Sampling.hpp
            include/Graphics/Scene.hpp
            include/Graphics/ShaderProgram.hpp
            include/Graphics/SurfaceMesh.hpp
            include/Graphics/Theme.hpp
            include/Graphics/TriangleMesh.hpp
            include/Graphics/Viewport2D.hpp
            include/Graphics/Viewport3D.hpp
            include/Graphics/Widgets.hpp
)

target_link_libraries(NURBS.Graphics
    PUBLIC
        glm::glm
        glad
        imgui
    PRIVATE
        glfw
        OpenGL::GL
)

# glad declares the GL API itself and refuses to compile after the system
# GL/gl.h, which GLFW includes unless told not to.
target_compile_definitions(NURBS.Graphics PRIVATE GLFW_INCLUDE_NONE)

target_compile_features(NURBS.Graphics PUBLIC cxx_std_23)
set_target_properties(NURBS.Graphics PROPERTIES CXX_EXTENSIONS OFF)
