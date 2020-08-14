#pragma once

#include <anton/array.hpp>
#include <anton/gizmo/common.hpp>
#include <anton/optional.hpp>

namespace anton::gizmo {
    [[nodiscard]] Array<math::Vec3> generate_filled_circle(f32 radius, i32 vertex_count);
    [[nodiscard]] Array<math::Vec3> generate_cube(f32 edge_length);
    [[nodiscard]] Array<math::Vec3> generate_icosphere(f32 radius, i64 subdivision_level);

    [[nodiscard]] Optional<f32> intersect_cube(math::Ray ray, f32 edge_length, math::Mat4 const& world_transform);
    [[nodiscard]] Optional<f32> intersect_sphere(math::Ray ray, f32 radius, math::Mat4 const& world_transform);
} // namespace anton::gizmo
