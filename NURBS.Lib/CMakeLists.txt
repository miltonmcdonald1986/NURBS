add_library(NURBS.Lib)
add_library(NURBS::Lib ALIAS NURBS.Lib)

target_sources(NURBS.Lib
    PRIVATE
        src/NURBS.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS include
        FILES
            include/NURBS/NURBS.hpp
            include/NURBS/bernstein.hpp
            include/NURBS/bezier.hpp
            include/NURBS/point.hpp
            include/NURBS/power_basis.hpp
)

target_include_directories(NURBS.Lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(NURBS.Lib PUBLIC cxx_std_23)
set_target_properties(NURBS.Lib PROPERTIES CXX_EXTENSIONS OFF)
