include(FetchContent)

FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.18.0.tar.gz
    URL_HASH SHA256=6e3191c1455468b3fc35a417fb565c1c5071aee1b7e7f85e30cf48a98d37d8b5
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(NURBS.Lib.Tests)

target_sources(NURBS.Lib.Tests
    PRIVATE
        NURBS.Tests.cpp
        bernstein.Tests.cpp
        bezier.Tests.cpp
        point.Tests.cpp
        power_basis.Tests.cpp
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
