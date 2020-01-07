#ifndef ANTON_GIZMO_ARROW_3D_HPP_INCLUDE
#define ANTON_GIZMO_ARROW_3D_HPP_INCLUDE

#include <anton_gizmo/base_types.hpp>
#include <optional>

namespace anton {
    // Arrow head style
    enum class Arrow_3D_Style {
        cone,
        cube,
    };

    struct Arrow_3D {
        Arrow_3D_Style draw_style;
        // Size in pixels
        uint32_t size;
    };

    // Returns how many elements a buffer should have at least to fit a gizmo with given number of vertices.
    // TODO: Explain what is the vertex_count
    uint32_t get_required_buffer_size_arrow_3d(Arrow_3D_Style const style, uint32_t const vertex_count);

    // Writes vertex positions to the vertices buffer
    // The arrow is aligned with z = -1, i.e. it extends from (0, 0, 0) to (0, 0, -1).
    // The function writes 2 vertices for the gizmo line to vertices[0] and vertices[1] and then writes vertex positions of the head.
    // For cone this function writes:
    //   a triangle fan of vertex_count + 1 vertices in CCW order for the cone
    //   a triangle fan of vertex_count vertices in CCW order for the base
    // For cube it writes a triangle strip of 14 vertices in CCW order for the cube
    //
    // Returns the number of vertices written.
    uint32_t generate_arrow_3d_geometry(Arrow_3D_Style const style, uint32_t const vertex_count, float* vertices);

    // Tests the arrow for intersection and returns the distance to the intersection point along the ray if the ray intersects the bounding volumes.
    // world_transform and view_projection_matrix are 4x4 matrices in row-major layout
    std::optional<float> intersect_arrow_3d(Ray, Arrow_3D, float const* world_transform, float const* view_projection_matrix, uint32_t viewport_width,
                                            uint32_t viewport_height);
} // namespace anton

#endif // !ANTON_GIZMO_ARROW_3D_HPP_INCLUDE