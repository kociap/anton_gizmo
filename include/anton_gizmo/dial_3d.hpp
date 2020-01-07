#ifndef ANTON_GIZMO_DIAL_3D_HPP_INCLUDE
#define ANTON_GIZMO_DIAL_3D_HPP_INCLUDE

#include <anton_gizmo/base_types.hpp>
#include <anton_math/matrix4.hpp>
#include <optional>

namespace anton {
    struct Dial_3D {
        // size in pixels
        uint32_t size;
    };

    // Returns how many elements a buffer should have at least to fit a gizmo with given number of vertices.
    uint32_t get_required_buffer_size_dial_3d(uint32_t vertex_count);

    // Writes vertex positions, axes of rotations to align the line with the view and scale factors to preserve the width at the corners.
    // The circle lies on the plane z = 0.
    // The function writes 2 vertices per line vertex to later expand them into a wide line in a shader
    //   (example shader may be found in examples/).
    //
    // Returns the number of vertices written.
    uint32_t generate_dial_3d_geometry(uint32_t vertex_count, float* vertices, float* rotation_axes, float* scale_factors);

    // Tests the arrow for intersection and returns the distance to the intersection point along the ray if the ray intersects the bounding volumes.
    // world_transform and view_projection_matrix are 4x4 matrices in row-major layout
    std::optional<float> intersect_dial_3d(Ray, Dial_3D, float const* world_transform, float const* view_projection_matrix, uint32_t viewport_width,
                                           uint32_t viewport_height);
} // namespace anton

#endif // !ANTON_GIZMO_DIAL_3D_HPP_INCLUDE