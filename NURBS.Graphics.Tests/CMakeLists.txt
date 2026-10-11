# googletest is fetched by NURBS.Lib.Tests, which the root adds first.
add_executable(NURBS.Graphics.Tests)

target_sources(NURBS.Graphics.Tests
    PRIVATE
        Animation.Tests.cpp
        Bounds.Tests.cpp
        Canvas.Tests.cpp
        Gnomon.Tests.cpp
        Grid.Tests.cpp
        OrbitCamera.Tests.cpp
        Picking.Tests.cpp
        Sampling.Tests.cpp
        SurfaceMesh.Tests.cpp
)

# For the private Gnomon.hpp and Grid.hpp.
target_include_directories(NURBS.Graphics.Tests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/NURBS.Graphics/src
)

target_link_libraries(NURBS.Graphics.Tests
    PRIVATE
        Graphics::Graphics
        GTest::gtest_main
)

set_target_properties(NURBS.Graphics.Tests PROPERTIES CXX_EXTENSIONS OFF)

include(GoogleTest)
gtest_discover_tests(NURBS.Graphics.Tests)
