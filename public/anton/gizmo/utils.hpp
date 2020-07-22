#pragma once

#include <anton/gizmo/common.hpp>

namespace anton::gizmo {
    // world_transform and projection_matrix are 4x4 matrices in row-major layout
    f32 compute_scale(f32 const* world_transform, u32 target_size, f32 const* projection_mat, u32 viewport_width, u32 viewport_height);
} // namespace anton::gizmo
