#pragma once

#include <anton/array.hpp>
#include <anton/math/mat4.hpp>
#include <anton/math/primitives.hpp>
#include <anton/optional.hpp>
#include <anton/types.hpp>

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
    // Generates the geometry of a handle. The handle is directed towards -z and starts at (0, 0, 0).
    // The total length of the handle will be:
    //  - cone: shaft_length + cap_length
    //  - cube: shaft_length + 0.5 * cap_size
    //
    // Parameters:
    // arrow        - parameter struct that defines the shape and size of the geometry.
    // vertex_count - the number of vertices that comprise the base of the cone (in case cone is the draw_style).
    //
    // Returns:
    // Vertices of the triangles comprising the handle in CCW order.
    //
    [[nodiscard]] anton::Array<math::Vec3> generate_arrow_3d_geometry(Arrow_3D const& arrow, i32 vertex_count);

    // intersect_arrow_3d
    // Perform an intersection test of a ray against the bounding volumes of the arrow.
    // The arrows is located at (0, 0, 0) and is aligned with the -z axis before being transformed into world space.
    //
    // Parameters:
    // ray             - a world space ray to test against.
    // arrow           - parameter struct that defines the shape and size of the bounding volumes.
    //                   Should be identical to that passed to generate_arrow_3d_geometry.
    // world_transform - a transform to the world space. The transform must consist of
    //                   translation, rotation and uniform scale only.
    //
    // Returns:
    // Distance along ray's direction to the intersection point or null_optional if no intersection occured.
    //
    [[nodiscard]] Optional<f32> intersect_arrow_3d(math::Ray ray, Arrow_3D const& arrow, math::Mat4 const& world_transform);
} // namespace anton::gizmo
