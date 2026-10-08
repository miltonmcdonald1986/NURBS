cmake_minimum_required(VERSION 3.25)

project(NURBS LANGUAGES CXX)

option(NURBS_BUILD_TESTS "Build NURBS.Lib.Tests" ON)
option(NURBS_BUILD_VIEWER "Build NURBS.Viewer" OFF)

add_subdirectory(NURBS.Lib)

if(NURBS_BUILD_TESTS OR NURBS_BUILD_VIEWER)
    include(FetchContent)

    FetchContent_Declare(glm
        URL https://github.com/g-truc/glm/archive/refs/tags/1.0.3.tar.gz
        URL_HASH SHA256=6775e47231a446fd086d660ecc18bcd076531cfedd912fbd66e576b118607001
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLM_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(glm)
endif()

if(NURBS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(NURBS.Lib.Tests)
endif()

if(NURBS_BUILD_VIEWER)
    add_subdirectory(NURBS.Viewer)
endif()
