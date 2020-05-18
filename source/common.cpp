#include <anton/gizmo/common.hpp>

#include <anton/math/matrix4.hpp>
#include <anton/math/vector2.hpp>
#include <anton/math/vector3.hpp>
#include <anton/math/vector4.hpp>

namespace anton::gizmo {
    Ray screen_to_ray(f32 const* const inv_view_ptr, f32 const* const inv_projection_ptr, i32 screen_width, i32 screen_height, f32* const point_ptr) {
        math::Matrix4 const inv_view = math::Matrix4(inv_view_ptr);
        math::Matrix4 const inv_projection = math::Matrix4(inv_projection_ptr);

        math::Vector2 const point = math::Vector2(point_ptr[0], point_ptr[1]);
        // Transform screen point to normalized -1..1 coordinates
        f32 const normalized_x = 2.0f * point.x / static_cast<f32>(screen_width) - 1.0f;
        f32 const normalized_y = 2.0f * point.y / static_cast<f32>(screen_height) - 1.0f;

        math::Vector4 const ray_start = math::Vector4(normalized_x, normalized_y, -1.0f, 1.0f);
        math::Vector4 const ray_end = math::Vector4(normalized_x, normalized_y, 0.0f, 1.0f);

        math::Vector4 const ray_start_hg = inv_projection * ray_start;
        math::Vector4 const ray_end_hg = inv_projection * ray_end;

        math::Vector3 ray_start_homogenized = math::Vector3(ray_start_hg);
        math::Vector3 ray_end_homogenized = math::Vector3(ray_end_hg);

        if (ray_start_hg.w != 0.0f) {
            ray_start_homogenized /= ray_start_hg.w;
        }

        if (ray_end_hg.w != 0.0f) {
            ray_end_homogenized /= ray_end_hg.w;
        }

        math::Vector3 const ray_start_world_space = math::Vector3(inv_view * math::Vector4(ray_start_homogenized, 1.0f));
        math::Vector3 const ray_direction_view_space = ray_end_homogenized - ray_start_homogenized;
        math::Vector3 const ray_direction_world_space = math::Vector3(inv_view * math::Vector4(ray_direction_view_space, 0.0f));
        return {math::normalize(ray_start_world_space), math::normalize(ray_direction_world_space)};
    }
} // namespace anton::gizmo
