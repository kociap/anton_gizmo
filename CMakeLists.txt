cmake_minimum_required(VERSION 3.12)

project(anton_gizmo CXX)
set(CMAKE_CXX_STANDARD 17)

add_library(anton_gizmo)

set(ANTON_PUBLIC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_gizmo/arrow_3d.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_gizmo/base_types.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_gizmo/dial_3d.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_gizmo/gizmo.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_gizmo/utils.hpp"
    
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/math.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/matrix2.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/matrix3.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/matrix4.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/quaternion.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/transform.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/vector2.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/vector3.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/anton_math/vector4.hpp"
)
    
set(ANTON_PRIVATE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/source/anton_math/vector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/arrow_3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/base_types.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/dial_3d.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/intersection_tests.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/intersections_common.hpp"
)

target_sources(anton_gizmo
    PUBLIC ${ANTON_PUBLIC_FILES}
    PRIVATE ${ANTON_PRIVATE_FILES}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${ANTON_PRIVATE_FILES} ${ANTON_PUBLIC_FILES})

target_include_directories(anton_gizmo
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/source"
)
