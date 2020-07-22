#include <anton/gizmo/utils.hpp>

#include <anton/math/matrix4.hpp>
#include <anton/math/vector2.hpp>

namespace anton::gizmo {
    f32 compute_scale(f32 const* const world_transform_ptr, u32 const target_size, f32 const* const projection_mat_ptr, u32 const viewport_width,
                      u32 const viewport_height) {
        math::Matrix4 const projection_mat = math::Matrix4(projection_mat_ptr);
        math::Matrix4 const world_transform = math::Matrix4(world_transform_ptr);

        f32 const pixel_size = 1 / viewport_height;
        f32 const projected_w_comp = (projection_mat * world_transform[3]).w;
        return /* scale_basis * */ target_size * pixel_size * projected_w_comp;
    }
} // namespace anton::gizmo
