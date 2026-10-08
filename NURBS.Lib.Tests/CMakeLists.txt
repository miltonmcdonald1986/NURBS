include(FetchContent)

FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

FetchContent_Declare(glm
    URL https://github.com/g-truc/glm/archive/refs/tags/1.0.1.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glm)

add_executable(NURBS.Lib.Tests)

target_sources(NURBS.Lib.Tests
    PRIVATE
        NURBS.Tests.cpp
        bernstein.Tests.cpp
        horner1.Tests.cpp
)

target_link_libraries(NURBS.Lib.Tests
    PRIVATE
        NURBS::Lib
        GTest::gtest_main
        glm::glm
)

set_target_properties(NURBS.Lib.Tests PROPERTIES CXX_EXTENSIONS OFF)

include(GoogleTest)
gtest_discover_tests(NURBS.Lib.Tests)
