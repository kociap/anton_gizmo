#pragma once

#include <anton/gizmo/common.hpp>
#include <anton/math/matrix4.hpp>
#include <optional>

namespace anton::gizmo {
    struct Dial_3D {
        // size in pixels
        u32 size;
    };

    // Returns how many elements a buffer should have at least to fit a gizmo with given number of vertices.
    u32 get_required_buffer_size_dial_3d(u32 vertex_count);

    // Writes vertex positions, axes of rotations to align the line with the view and scale factors to preserve the width at the corners.
    // The circle lies on the plane z = 0.
    // The function writes 2 vertices per line vertex to later expand them into a wide line in a shader
    //   (example shader may be found in examples/).
    //
    // Returns the number of vertices written.
    u32 generate_dial_3d_geometry(u32 vertex_count, f32* vertices, f32* rotation_axes, f32* scale_factors);

    // Tests the arrow for intersection and returns the distance to the intersection point along the ray if the ray intersects the bounding volumes.
    // world_transform and view_projection_matrix are 4x4 matrices in row-major layout
    std::optional<f32> intersect_dial_3d(Ray, Dial_3D, f32 const* world_transform, f32 const* view_projection_matrix, u32 viewport_width, u32 viewport_height);
} // namespace anton::gizmo
