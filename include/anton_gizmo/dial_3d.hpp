#ifndef ANTON_GIZMO_DIAL_3D_HPP_INCLUDE
#define ANTON_GIZMO_DIAL_3D_HPP_INCLUDE

#include <anton_gizmo/base_types.hpp>
#include <anton_math/matrix4.hpp>
#include <anton_math/vector2.hpp>
#include <anton_math/vector3.hpp>
#include <optional>

namespace anton {
    struct Dial_3D {
        // size in pixels
        uint32_t size;
    };

    // Returns how many elements a buffer should have at least to fit a gizmo with given number of vertices.
    uint32_t get_required_buffer_size_dial_3d(uint32_t vertex_count);

    uint32_t generate_dial_3d_geometry(uint32_t vertex_count, Vector3* vertices, Vector3* rotation_axes, float* scale_factors);

    // Tests the arrow for intersection and returns the distance to the intersection point along the ray if the ray intersects the bounding volumes.
    std::optional<float> intersect_dial_3d(Ray, Dial_3D, Matrix4 world_transform, Matrix4 view_projection_matrix, Vector2 viewport_size);
} // namespace anton

#endif // !ANTON_GIZMO_DIAL_3D_HPP_INCLUDE