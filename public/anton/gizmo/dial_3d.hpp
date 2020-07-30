#pragma once

#include <anton/gizmo/common.hpp>
#include <anton/optional.hpp>
#include <anton/slice.hpp>

namespace anton::gizmo {
    struct Dial_3D {
        // size in pixels
        u32 size;
    };

    //
    // Returns how many elements a buffer should have at least to fit a gizmo with given number of vertices.
    //
    [[nodiscard]] u32 get_required_buffer_size_dial_3d(u32 vertex_count);

    // Writes vertex positions, axes of rotations to align the line with the view and scale factors to preserve the width at the corners.
    // The circle lies on the plane z = 0.
    // The function writes 2 vertices per line vertex to later expand them into a wide line in a shader
    //   (example shader may be found in examples/).
    //
    // Returns the number of vertices written.
    void generate_dial_3d_geometry(u32 vertex_count, Slice<math::Vector3> vertices);

    // Tests the arrow for intersection and returns the distance to the intersection point along the ray if the ray intersects the bounding volumes.
    //
    [[nodiscard]] Optional<f32> intersect_dial_3d(Ray ray, Dial_3D dial, math::Matrix4 world_transform, math::Matrix4 view_projection, u32 viewport_width,
                                                  u32 viewport_height);
} // namespace anton::gizmo
