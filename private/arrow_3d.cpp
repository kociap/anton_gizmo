#include <anton/gizmo/arrow_3d.hpp>

#include <anton/gizmo/utils.hpp>
#include <anton/math/matrix4.hpp>
#include <anton/math/quaternion.hpp>
#include <anton/math/vector2.hpp>
#include <anton/math/vector3.hpp>
#include <intersection_tests.hpp>

namespace anton::gizmo {
    static void generate_cone_geometry(u32 const vert_count, math::Vector3* const vertices) {
        vertices[0] = {0, 0, 0};
        vertices[1] = {0, 0, -0.8f};

        f32 angle = math::radians(360.0f / static_cast<f32>(vert_count));
        math::Vector3 rotation_axis = math::Vector3(0, 0, -1) * sin(angle / 2.0f);
        math::Quaternion rotation_quat(rotation_axis.x, rotation_axis.y, rotation_axis.z, cos(angle / 2.0f));

        math::Vector3 vertex = {0, 0.05f, 0};
        math::Quaternion rotated_vec = math::Quaternion(vertex.x, vertex.y, vertex.z, 0);
        math::Vector3* const base = vertices + 2 + vert_count + 1;
        math::Vector3* const cone = vertices + 2;
        base[0] = {0, 0, -0.8f};
        cone[0] = {0, 0, -1.0f};
        for (uint64_t i = 0; i <= uint64_t(vert_count); ++i) {
            rotated_vec = rotation_quat * rotated_vec * conjugate(rotation_quat);
            base[1 + uint64_t(vert_count) - i] = math::Vector3{rotated_vec.x, rotated_vec.y, -0.8f};
            cone[1 + i] = math::Vector3{rotated_vec.x, rotated_vec.y, -0.8f};
        }
    }

    static void generate_cube_geometry(math::Vector3* const vertices) {
        constexpr f32 half_size = 0.05f;
        math::Vector3 const offset = {0, 0, -1.0f + half_size};
        vertices[0] = {0, 0, 0};
        vertices[1] = {0, 0, -1.0f + half_size * 2};
        vertices[2] = offset + math::Vector3{half_size, -half_size, half_size};
        vertices[3] = offset + math::Vector3{-half_size, -half_size, half_size};
        vertices[4] = offset + math::Vector3{half_size, -half_size, -half_size};
        vertices[5] = offset + math::Vector3{-half_size, -half_size, -half_size};
        vertices[6] = offset + math::Vector3{-half_size, half_size, -half_size};
        vertices[7] = offset + math::Vector3{-half_size, -half_size, half_size};
        vertices[8] = offset + math::Vector3{-half_size, half_size, half_size};
        vertices[9] = offset + math::Vector3{half_size, -half_size, half_size};
        vertices[10] = offset + math::Vector3{half_size, half_size, half_size};
        vertices[11] = offset + math::Vector3{half_size, -half_size, -half_size};
        vertices[12] = offset + math::Vector3{half_size, half_size, -half_size};
        vertices[13] = offset + math::Vector3{-half_size, half_size, -half_size};
        vertices[14] = offset + math::Vector3{half_size, half_size, half_size};
        vertices[15] = offset + math::Vector3{-half_size, half_size, half_size};
    }

    u32 get_required_buffer_size_arrow_3d(Arrow_3D_Style const style, u32 const vertex_count) {
        switch (style) {
            case Arrow_3D_Style::cone: {
                return 2 + (vertex_count + 1) + vertex_count; // line, cone and base
            }

            case Arrow_3D_Style::cube: {
                return 16;
            }
        }
    }

    u32 generate_arrow_3d_geometry(Arrow_3D_Style const style, u32 const vertex_count, f32* const vertices) {
        switch (style) {
            case Arrow_3D_Style::cone: {
                generate_cone_geometry(vertex_count, reinterpret_cast<math::Vector3*>(vertices));
            } break;

            case Arrow_3D_Style::cube: {
                generate_cube_geometry(reinterpret_cast<math::Vector3*>(vertices));
            } break;
        }
        return get_required_buffer_size_arrow_3d(style, vertex_count);
    }

    std::optional<f32> intersect_arrow_3d(Ray const ray, Arrow_3D const arrow, f32 const* const world_transform_ptr,
                                          f32 const* const view_projection_matrix_ptr, u32 const viewport_width, u32 const viewport_height) {
        math::Matrix4 const world_transform = math::Matrix4(world_transform_ptr);
        f32 const scale = compute_scale(world_transform_ptr, arrow.size, view_projection_matrix_ptr, viewport_width, viewport_height);
        switch (arrow.draw_style) {
            case Arrow_3D_Style::cone: {
                std::optional<f32> result = std::nullopt;

                math::Vector3 const vertex1 = math::Vector3(world_transform * math::Vector4(0, 0, 0, 1));
                math::Vector3 const vertex2 = math::Vector3(world_transform * math::Vector4(0, 0, -0.8f * scale, 1));
                if (std::optional<Raycast_Hit> const hit = intersect_ray_cylinder(ray, vertex1, vertex2, 0.05f * scale);
                    hit.has_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                math::Vector3 const vertex = math::Vector3(world_transform * math::scale(scale) * math::Vector4(0.0f, 0.0f, -1.05f, 1.0f));
                math::Vector3 const direction = math::Vector3(world_transform * math::Vector4(0.0f, 0.0f, 1.0f, 0.0f));
                // 0.2 height, 0.05 radius
                f32 const angle_cos = 0.970143f;
                f32 const height = 0.3f * scale;
                if (std::optional<Raycast_Hit> const hit = intersect_ray_cone(ray, vertex, direction, angle_cos, height);
                    hit.has_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                return result;
            }

            case Arrow_3D_Style::cube: {
                std::optional<f32> result = std::nullopt;

                math::Vector3 const vertex1 = math::Vector3(world_transform * math::Vector4(0, 0, 0, 1));
                math::Vector3 const vertex2 = math::Vector3(world_transform * math::Vector4(0, 0, -0.8f * scale, 1));
                if (std::optional<Raycast_Hit> const hit = intersect_ray_cylinder(ray, vertex1, vertex2, 0.05f * scale);
                    hit.has_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                OBB cube_bounding_vol;
                cube_bounding_vol.local_x = math::Vector3(world_transform * math::Vector4(1.0f, 0.0f, 0.0f, 0.0f));
                cube_bounding_vol.local_y = math::Vector3(world_transform * math::Vector4(0.0f, 1.0f, 0.0f, 0.0f));
                cube_bounding_vol.local_z = math::Vector3(world_transform * math::Vector4(0.0f, 0.0f, -1.0f, 0.0f));
                cube_bounding_vol.halfwidths = {0.1f * scale, 0.1f * scale, 0.1f * scale};
                cube_bounding_vol.center = math::get_translation(world_transform) + cube_bounding_vol.local_z * 0.95f * scale;
                if (std::optional<Raycast_Hit> const hit = intersect_ray_obb(ray, cube_bounding_vol); hit.has_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                return result;
            }
        }
    }
} // namespace anton::gizmo
