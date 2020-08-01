#pragma once

#include <anton/math/matrix4.hpp>
#include <anton/math/vector2.hpp>
#include <anton/math/vector3.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    struct Ray {
        math::Vector3 origin;
        math::Vector3 direction;
    };

    // target_size - pixels
    inline f32 compute_scale(math::Matrix4 const& world_transform, u32 const target_size, math::Matrix4 const& projection, math::Vector2 const viewport_size) {
        f32 const pixel_size = 1 / viewport_size.y;
        f32 const projected_w_comp = (projection * world_transform[3]).w;
        return /* scale_basis * */ target_size * pixel_size * projected_w_comp;
    }
} // namespace anton::gizmo
