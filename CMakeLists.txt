cmake_minimum_required(VERSION 3.25)

project(NURBS LANGUAGES CXX)

option(NURBS_BUILD_TESTS "Build NURBS.Lib.Tests" ON)

add_subdirectory(NURBS.Lib)

if(NURBS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(NURBS.Lib.Tests)
endif()
