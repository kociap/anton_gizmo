#pragma once

#include <anton/math/vector3.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    struct Ray {
        math::Vector3 origin;
        math::Vector3 direction;
    };

    // Unprojects a point in screen coordinates to world coordinates. Assumes OpenGL NDC.
    // point is a 2D point (2 component vector)
    // inverse_view and inverse_projection are 4x4 matrices in row-major layout
    Ray screen_to_ray(f32 const* inverse_view, f32 const* inverse_projection, i32 screen_width, i32 screen_height, f32* point);
} // namespace anton::gizmo
