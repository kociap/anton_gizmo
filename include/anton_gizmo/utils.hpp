#ifndef ANTON_GIZMO_UTILS_HPP_INCLUDE
#define ANTON_GIZMO_UTILS_HPP_INCLUDE

#include <anton_gizmo/base_types.hpp>

namespace anton {
    float compute_scale(Matrix4 world_transform, uint32_t target_size, Matrix4 pers_mat, Vector2 viewport_size);
}

#endif // !ANTON_GIZMO_UTILS_HPP_INCLUDE
