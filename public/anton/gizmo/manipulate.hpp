#ifndef ANTON_GIZMO_MANIPULATE_HPP_INCLUDE
#define ANTON_GIZMO_MANIPULATE_HPP_INCLUDE

#include <stdint.h>

namespace anton {
    struct Vec3 {
        float x;
        float y;
        float z;
    };

    Vec3 translate(float const* inv_view_projection_matrix, int32_t screen_width, int32_t screen_height, float const* initial_point,
                   float const* currrent_point, float const* plane_normal, float plane_distance);
} // namespace anton

#endif // !ANTON_GIZMO_MANIPULATE_HPP_INCLUDE