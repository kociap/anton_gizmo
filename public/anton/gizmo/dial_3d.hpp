#pragma once

#include <anton/array.hpp>
#include <anton/math/mat4.hpp>
#include <anton/math/primitives.hpp>
#include <anton/optional.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    struct Dial_3D {
        // Radius of the dial.
        f32 major_radius;
        // Thickness of the dial.
        f32 minor_radius;
    };

    // generate_dial_3d_geometry
    // Generates the geometry of a dial. The dial lies in a plane with its normal directed towards -z and is centered at (0, 0, 0).
    //
    // Parameters:
    // dial - Parameter struct that defines the size of the geometry.
    // vertex_count_major - the number of vertices that comprise the large circle of the dial.
    // vertex_count_minor - the number of vertices that comprise the small circle of the dial.
    //
    // Returns:
    // Vertices of the triangles comprising the dial in CCW order.
    //
    [[nodiscard]] Array<math::Vec3> generate_dial_3d_geometry(Dial_3D const& dial, i32 vertex_count_major, i32 vertex_count_minor);

    // intersect_dial_3d
    // Perform an intersection test of a ray against the bounding volumes of the dial.
    // The dial is located at (0, 0, 0) and is aligned with the -z axis before being transformed into world space.
    //
    // Parameters:
    // ray - A world space ray to test against.
    // dial - Parameter struct that defines the size of the bounding volumes. Should be identical to that passed generate_dial_3d_geometry.
    // world_transform - A translation-rotation transform to world space.
    //
    // Returns:
    // Distance along ray's direction to the intersection point or null_optional if no intersection occured.
    //
    [[nodiscard]] Optional<f32> intersect_dial_3d(math::Ray ray, Dial_3D const& dial, math::Mat4 const& world_transform);
} // namespace anton::gizmo
