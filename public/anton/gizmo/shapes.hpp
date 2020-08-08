#pragma once

#include <anton/array.hpp>
#include <anton/math/vec3.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    [[nodiscard]] Array<math::Vec3> generate_cube(f32 edge_length);
    [[nodiscard]] Array<math::Vec3> generate_icosphere(f32 radius, i64 subdivision_level);
} // namespace anton::gizmo
