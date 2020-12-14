#pragma once

#include <anton/array.hpp>
#include <anton/gizmo/common.hpp>
#include <anton/optional.hpp>

namespace anton::gizmo {
    [[nodiscard]] Array<math::Vec3> generate_filled_circle(f32 radius, i32 vertex_count);
    [[nodiscard]] Array<math::Vec3> generate_cube(f32 edge_length);
    [[nodiscard]] Array<math::Vec3> generate_icosphere(f32 radius, i64 subdivision_level);

    // intersect_cube
    // Perform an intersection test of a ray against a cube.
    // The cube's center is located at (0, 0, 0) and is aligned with the axes before
    // being transformed into world space.
    //
    // Parameters:
    // ray - A world space ray to test against.
    // edge_length - The length of the cube's edge.
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
    // ray - A world space ray to test against.
    // radius - Radius of the sphere.
    // world_transform - A translation-rotation transform to world space.
    //
    // Returns:
    // Distance along ray's direction to the intersection point or null_optional if no intersection occured.
    //
    [[nodiscard]] Optional<f32> intersect_sphere(math::Ray ray, f32 radius, math::Mat4 const& world_transform);
} // namespace anton::gizmo
