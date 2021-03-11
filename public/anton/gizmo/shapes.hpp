#pragma once

#include <anton/array.hpp>
#include <anton/gizmo/common.hpp>
#include <anton/optional.hpp>

namespace anton::gizmo {
    // generate_filled_circle
    // Generates a filled circle centered at origin and facing in the direction of the normal.
    //
    // Parameters:
    // origin       - the initial position of the circle.
    // normal       - the normal of the circle. Must be normalized and non-zero.
    // radius       - the radius of the circle.
    // vertex_count - the number of vertices that the circle will consist of.
    //
    // Returns:
    // An array containing a triangle list.
    //
    [[nodiscard]] Array<math::Vec3> generate_filled_circle(math::Vec3 const& origin, math::Vec3 const& normal, f32 radius, i32 vertex_count);

    // generate_square
    // Generates a square centered at origin and facing in the direction of the normal aligned with up.
    //
    // Parameters:
    // origin      - the initial position of the square.
    // normal      - the normal of the square. Must be normalized and non-zero.
    // up          - the up vector of the square. Must be normalized and non-zero.
    // edge_length - the length of the edge of the square.
    //
    // Returns:
    // An array containing a triangle list.
    //
    [[nodiscard]] Array<math::Vec3> generate_square(math::Vec3 const& origin, math::Vec3 const& normal, math::Vec3 const& up, f32 edge_length);

    [[nodiscard]] Array<math::Vec3> generate_cube(f32 edge_length);

    // generate_icosphere
    // Generates an icosphere centered at (0, 0, 0).
    //
    // Parameters:
    // radius            - the radius of the icosphere.
    // subdivision_level - the level of subdivision of the icosphere. Each level increases detail
    //                     and smoothness at the expense of quadrupling the triangle count.
    //
    // Returns:
    // An array containing a triangle list.
    //
    [[nodiscard]] Array<math::Vec3> generate_icosphere(f32 radius, i64 subdivision_level);

    // intersect_cube
    // Perform an intersection test of a ray against a cube.
    // The cube's center is located at (0, 0, 0) and is aligned with the axes before
    // being transformed into world space.
    //
    // Parameters:
    // ray             - A world space ray to test against.
    // edge_length     - The length of the cube's edge.
    // world_transform - A translation-rotation transform to world space.
    //
    // Returns:
    // Distance along ray's direction to the intersection point or null_optional if no intersection occured.
    //
    [[nodiscard]] Optional<f32> intersect_cube(math::Ray ray, f32 edge_length, math::Mat4 const& world_transform);

    // intersect_sphere
    // Perform an intersection test of a ray against a sphere.
    // The sphere's center is located at (0, 0, 0) before being transformed into world space.
    //
    // Parameters:
    // ray             - A world space ray to test against.
    // radius          - Radius of the sphere.
    // world_transform - A translation-rotation transform to world space.
    //
    // Returns:
    // Distance along ray's direction to the intersection point or null_optional if no intersection occured.
    //
    [[nodiscard]] Optional<f32> intersect_sphere(math::Ray ray, f32 radius, math::Mat4 const& world_transform);
} // namespace anton::gizmo
