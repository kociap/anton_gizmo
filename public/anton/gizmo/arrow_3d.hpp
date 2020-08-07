#pragma once

#include <anton/array.hpp>
#include <anton/gizmo/common.hpp>
#include <anton/optional.hpp>

namespace anton::gizmo {
    // Arrow head style
    enum class Arrow_3D_Style {
        cone,
        cube,
    };

    struct Arrow_3D {
        Arrow_3D_Style draw_style;
        // For cone it's the diameter of the base of the cone.
        // For cube it's the length of the edge of the cube.
        f32 cap_size;
        // For cone it's the height of the cone.
        // Ignored for cube.
        f32 cap_length;
        f32 shaft_length;
        f32 shaft_diameter;
    };

    // generate_arrow_3d_geometry
    // Generates the geometry of a handle. The handle is directed towards -z.
    // The total length of the handle will be:
    //  - cone: shaft_length + cap_length
    //  - cube: shaft_length + 0.5 * cap_size
    //
    // Returns:
    // Vertices of the triangles comprising the handle in CCW order.
    //
    [[nodiscard]] anton::Array<math::Vector3> generate_arrow_3d_geometry(Arrow_3D const& arrow, i32 vertex_count);

    // intersect_arrow_3d
    // Tests the arrow for intersection and returns the distance to the intersection point along the ray if the ray intersects the bounding volumes.
    //
    [[nodiscard]] Optional<f32> intersect_arrow_3d(math::Ray ray, Arrow_3D const& arrow, math::Matrix4 const& world_transform);
} // namespace anton::gizmo
