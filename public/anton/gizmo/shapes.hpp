#pragma once

#include <anton/array.hpp>
#include <anton/math/mat4.hpp>
#include <anton/math/primitives.hpp>
#include <anton/math/vec3.hpp>
#include <anton/optional.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    // generate_filled_circle
    // Generates a filled circle of radius 1.0 centered at (0, 0, 0) with normal along -z.
    //
    // Parameters:
    // vertex_count - the number of vertices that the circle will consist of.
    //
    // Returns:
    // An array containing a triangle list in CCW order when looking at the circle in the direction of +z.
    //
    [[nodiscard]] Array<math::Vec3> generate_filled_circle(i32 vertex_count);

    // generate_square
    // Generates a square centered at (0, 0, 0) with normal along -z. The edges are of length 1.0 and are aligned with the x and y axes.
    //
    // Returns:
    // An array containing a triangle list in CCW order when looking at the square in the direction of +z.
    //
    [[nodiscard]] Array<math::Vec3> generate_square();

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

    // intersect_circle
    // Perform an intersection test of a ray against a circle.
    // Before being transformed using world_transform, the circle is centered at (0, 0, 0)
    // with normal along -z and radius 1.0.
    //
    // Parameters:
    // ray             - a world space ray to test against.
    // world_transform - a transform to world space.
    //
    // Returns:
    // Distance along ray's direction to the intersection point or null_optional if no intersection occured.
    //
    [[nodiscard]] Optional<f32> intersect_circle(math::Ray const& ray, math::Mat4 const& world_transform);

    // intersect_square
    // Perform an intersection test of a ray against a square.
    // Before being transformed using world_transform, the square is centered at (0, 0, 0)
    // with normal along -z and edges of length 1.0 along x and y axes.
    //
    // Parameters:
    // ray             - a world space ray to test against.
    // world_transform - a transform to world space.
    //
    // Returns:
    // Distance along ray's direction to the intersection point or null_optional if no intersection occured.
    //
    [[nodiscard]] Optional<f32> intersect_square(math::Ray const& ray, math::Mat4 const& world_transform);

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
    [[nodiscard]] Optional<f32> intersect_cube(math::Ray const& ray, f32 edge_length, math::Mat4 const& world_transform);

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
    [[nodiscard]] Optional<f32> intersect_sphere(math::Ray const& ray, f32 radius, math::Mat4 const& world_transform);
} // namespace anton::gizmo
