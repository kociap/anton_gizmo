cmake_minimum_required(VERSION 3.13)

string(FIND "${CMAKE_CXX_COMPILER}" "clang++" ANTON_GIZMO_COMPILER_CLANGPP)
if(ANTON_GIZMO_COMPILER_CLANGPP GREATER -1)
  set(ANTON_GIZMO_COMPILER_CLANGPP ON)
else()
  set(ANTON_GIZMO_COMPILER_CLANGPP OFF)
endif()

if(ANTON_GIZMO_COMPILER_CLANGPP)
    set(ANTON_GIZMO_COMPILE_FLAGS
        -Wall -Wextra -pedantic
        -ferror-limit=1
        -Wnon-virtual-dtor
        -Werror=return-type
        -fno-char8_t # Yea, just no.
        -fno-rtti 
        -fno-exceptions
    )
endif()

project(anton_gizmo)

add_subdirectory(ext/anton_math)

add_library(anton_gizmo)
set_target_properties(anton_gizmo PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
target_compile_options(anton_gizmo PRIVATE ${ANTON_GIZMO_COMPILE_FLAGS})
target_link_libraries(anton_gizmo PUBLIC anton_math)
target_sources(anton_gizmo
    PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/arrow_3d.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/common.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/dial_3d.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/gizmo.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/utils.hpp"

    PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}/private/arrow_3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/private/common.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/private/dial_3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/private/intersection_tests.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/private/utils.cpp"
)
target_include_directories(anton_gizmo
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/public"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/private"
)
