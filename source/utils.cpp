#include <anton_gizmo/utils.hpp>

#include <anton_math/matrix4.hpp>
#include <anton_math/vector2.hpp>

namespace anton {
    float compute_scale(Matrix4 const world_transform, u32 const target_size, Matrix4 const pers_mat, uint32_t const viewport_width,
                        uint32_t const viewport_height) {
        Vector2 const viewport_size = Vector2(viewport_width, viewport_height);
        float const pixel_size = 1 / viewport_size.y;
        float const projected_w_comp = (world_transform[3] * pers_mat).w;
        return /* scale_basis * */ target_size * pixel_size * projected_w_comp;
    }
} // namespace anton
