#ifndef ANTON_GIZMO_BASE_TYPES_HPP_INCLUDE
#define ANTON_GIZMO_BASE_TYPES_HPP_INCLUDE

#include <anton_math/matrix4.hpp>
#include <anton_math/vector2.hpp>
#include <anton_math/vector3.hpp>
#include <stdint.h>

namespace anton {
    struct Ray {
        Vector3 origin;
        Vector3 direction;
    };

    Ray screen_to_ray(Matrix4 inv_view, Matrix4 inv_projection, int32_t screen_width, int32_t screen_height, Vector2 point);
} // namespace anton

#endif // !ANTON_GIZMO_BASE_TYPES_HPP_INCLUDE
