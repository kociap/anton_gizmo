#pragma once

#include <anton/array.hpp>
#include <anton/math/vector3.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    [[nodiscard]] Array<math::Vec3> generate_icosphere(f32 radius, i64 subdivision_level);
}
