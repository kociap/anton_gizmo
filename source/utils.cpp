#include <anton_gizmo/utils.hpp>

#include <anton_math/matrix4.hpp>
#include <anton_math/vector2.hpp>

namespace anton {
    float compute_scale(float const* const world_transform_ptr, u32 const target_size, float const* const projection_mat_ptr, uint32_t const viewport_width,
                        uint32_t const viewport_height) {
        Matrix4 const projection_mat = Matrix4(projection_mat_ptr);
        Matrix4 const world_transform = Matrix4(world_transform_ptr);

        float const pixel_size = 1 / viewport_height;
        float const projected_w_comp = (world_transform[3] * projection_mat).w;
        return /* scale_basis * */ target_size * pixel_size * projected_w_comp;
    }
} // namespace anton
