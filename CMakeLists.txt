cmake_minimum_required(VERSION 3.13)

add_subdirectory(ext/anton_math)

project(anton_gizmo CXX)
set(CMAKE_CXX_STANDARD 17)

add_library(anton_gizmo)

target_sources(anton_gizmo
    PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton/gizmo/arrow_3d.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton/gizmo/common.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton/gizmo/dial_3d.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton/gizmo/gizmo.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton/gizmo/utils.hpp"
    
    PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}/source/arrow_3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/common.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/dial_3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/intersection_tests.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/utils.cpp"
)

target_include_directories(anton_gizmo
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/source"
)

target_link_libraries(anton_gizmo PUBLIC anton_math)
