#ifndef ANTON_GIZMO_ARROW_3D_HPP_INCLUDE
#define ANTON_GIZMO_ARROW_3D_HPP_INCLUDE

#include <anton_gizmo/base_types.hpp>
#include <anton_math/matrix4.hpp>
#include <anton_math/vector2.hpp>
#include <anton_math/vector3.hpp>
#include <optional>
#include <vector>

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

    uint32_t generate_arrow_3d_geometry(Arrow_3D_Style const style, uint32_t const vertex_count, Vector3* vertices);
    // Tests the arrow for intersection and returns the distance to the intersection point along the ray if the ray intersects the bounding volumes.
    std::optional<float> intersect_arrow_3d(Ray, Arrow_3D, Matrix4 world_transform, Matrix4 view_projection_matrix, Vector2 viewport_size);
} // namespace anton

#endif // !ANTON_GIZMO_ARROW_3D_HPP_INCLUDE