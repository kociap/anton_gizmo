#ifndef ANTON_GIZMO_UTILS_HPP_INCLUDE
#define ANTON_GIZMO_UTILS_HPP_INCLUDE

#include <anton_gizmo/base_types.hpp>

namespace anton {
    // world_transform and projection_matrix are 4x4 matrices in row-major layout
    float compute_scale(float const* world_transform, uint32_t target_size, float const* projection_mat, uint32_t viewport_width, uint32_t viewport_height);
} // namespace anton

#endif // !ANTON_GIZMO_UTILS_HPP_INCLUDE
