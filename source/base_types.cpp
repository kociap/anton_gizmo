#include <anton_gizmo/base_types.hpp>

#include <anton_math/matrix4.hpp>
#include <anton_math/vector2.hpp>
#include <anton_math/vector3.hpp>
#include <anton_math/vector4.hpp>

namespace anton {
    Ray screen_to_ray(Matrix4 inv_view, Matrix4 inv_projection, int32_t screen_width, int32_t screen_height, float* const point_ptr) {
        Vector2 const point = Vector2(point_ptr[0], point_ptr[1]);
        // Transform screen point to normalized -1..1 coordinates
        float const normalized_x = 2.0f * point.x / static_cast<float>(screen_width) - 1.0f;
        float const normalized_y = 2.0f * point.y / static_cast<float>(screen_height) - 1.0f;

        Vector4 const ray_start = Vector4(normalized_x, normalized_y, -1.0f, 1.0f);
        Vector4 const ray_end = Vector4(normalized_x, normalized_y, 0.0f, 1.0f);

        Vector4 const ray_start_hg = ray_start * inv_projection;
        Vector4 const ray_end_hg = ray_end * inv_projection;

        Vector3 ray_start_homogenized = Vector3(ray_start_hg);
        Vector3 ray_end_homogenized = Vector3(ray_end_hg);

        if (ray_start_hg.w != 0.0f) {
            ray_start_homogenized /= ray_start_hg.w;
        }
        if (ray_end_hg.w != 0.0f) {
            ray_end_homogenized /= ray_end_hg.w;
        }

        Vector3 ray_direction_view_space = ray_end_homogenized - ray_start_homogenized;
        ray_direction_view_space.normalize();

        Vector3 ray_start_world_space = Vector3(Vector4(ray_start_homogenized, 1.0f) * inv_view);
        Vector3 ray_direction_world_space = Vector3(Vector4(ray_direction_view_space, 0.0f) * inv_view);
        ray_direction_world_space.normalize();

        return {ray_start_world_space, ray_direction_world_space};
    }
} // namespace anton
