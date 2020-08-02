cmake_minimum_required(VERSION 3.15)

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

# Add anton_core
FetchContent_Declare(
    anton_core
    GIT_REPOSITORY https://github.com/kociap/anton_core.git
)
FetchContent_MakeAvailable(anton_core)

add_library(anton_gizmo)
set_target_properties(anton_gizmo PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
target_compile_options(anton_gizmo PRIVATE ${ANTON_GIZMO_COMPILE_FLAGS})
target_link_libraries(anton_gizmo PUBLIC anton_core)
target_sources(anton_gizmo
    PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/arrow_3d.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/common.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/dial_3d.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/gizmo.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/public/anton/gizmo/manipulate.hpp"
    
    PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}/private/arrow_3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/private/dial_3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/private/intersection_tests.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/private/utils.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/private/manipulate.cpp"
)
target_include_directories(anton_gizmo
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/public"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/private"
)
