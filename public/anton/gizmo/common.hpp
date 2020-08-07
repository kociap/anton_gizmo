#pragma once

#include <anton/math/mat4.hpp>
#include <anton/math/primitives.hpp>
#include <anton/math/vec2.hpp>
#include <anton/math/vec3.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    // target_size - pixels
    inline f32 compute_scale(math::Mat4 const& world_transform, u32 const target_size, math::Mat4 const& projection, math::Vec2 const viewport_size) {
        f32 const pixel_size = 1 / viewport_size.y;
        f32 const projected_w_comp = (projection * world_transform[3]).w;
        return /* scale_basis * */ target_size * pixel_size * projected_w_comp;
    }
} // namespace anton::gizmo
