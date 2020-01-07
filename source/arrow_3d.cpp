#include <anton_gizmo/arrow_3d.hpp>

#include <anton_gizmo/utils.hpp>
#include <anton_math/matrix4.hpp>
#include <anton_math/quaternion.hpp>
#include <anton_math/vector2.hpp>
#include <anton_math/vector3.hpp>
#include <intersection_tests.hpp>

namespace anton {
    static void generate_cone_geometry(uint32_t const vert_count, Vector3* const vertices) {
        vertices[0] = {0, 0, 0};
        vertices[1] = {0, 0, -0.8f};

        float angle = radians(360.0f / static_cast<float>(vert_count));
        Vector3 rotation_axis = Vector3(0, 0, -1) * sin(angle / 2.0f);
        Quaternion rotation_quat(rotation_axis.x, rotation_axis.y, rotation_axis.z, cos(angle / 2.0f));

        Vector3 vertex = {0, 0.05f, 0};
        Quaternion rotated_vec = Quaternion(vertex.x, vertex.y, vertex.z, 0);
        Vector3* const base = vertices + 2 + vert_count + 1;
        Vector3* const cone = vertices + 2;
        base[0] = {0, 0, -0.8f};
        cone[0] = {0, 0, -1.0f};
        for (uint64_t i = 0; i <= uint64_t(vert_count); ++i) {
            rotated_vec = rotation_quat * rotated_vec * conjugate(rotation_quat);
            base[1 + uint64_t(vert_count) - i] = Vector3{rotated_vec.x, rotated_vec.y, -0.8f};
            cone[1 + i] = Vector3{rotated_vec.x, rotated_vec.y, -0.8f};
        }
    }

    static void generate_cube_geometry(Vector3* const vertices) {
        constexpr float half_size = 0.05f;
        Vector3 const offset = {0, 0, -1.0f + half_size};
        vertices[0] = {0, 0, 0};
        vertices[1] = {0, 0, -1.0f + half_size * 2};
        vertices[2] = offset + Vector3{half_size, -half_size, half_size};
        vertices[3] = offset + Vector3{-half_size, -half_size, half_size};
        vertices[4] = offset + Vector3{half_size, -half_size, -half_size};
        vertices[5] = offset + Vector3{-half_size, -half_size, -half_size};
        vertices[6] = offset + Vector3{-half_size, half_size, -half_size};
        vertices[7] = offset + Vector3{-half_size, -half_size, half_size};
        vertices[8] = offset + Vector3{-half_size, half_size, half_size};
        vertices[9] = offset + Vector3{half_size, -half_size, half_size};
        vertices[10] = offset + Vector3{half_size, half_size, half_size};
        vertices[11] = offset + Vector3{half_size, -half_size, -half_size};
        vertices[12] = offset + Vector3{half_size, half_size, -half_size};
        vertices[13] = offset + Vector3{-half_size, half_size, -half_size};
        vertices[14] = offset + Vector3{half_size, half_size, half_size};
        vertices[15] = offset + Vector3{-half_size, half_size, half_size};
    }

    uint32_t get_required_buffer_size_arrow_3d(Arrow_3D_Style const style, uint32_t const vertex_count) {
        switch (style) {
            case Arrow_3D_Style::cone: {
                return 2 + (vertex_count + 1) + vertex_count; // line, cone and base
            }

            case Arrow_3D_Style::cube: {
                return 16;
            }
        }
    }

    uint32_t generate_arrow_3d_geometry(Arrow_3D_Style const style, uint32_t const vertex_count, float* const vertices) {
        switch (style) {
            case Arrow_3D_Style::cone: {
                generate_cone_geometry(vertex_count, reinterpret_cast<Vector3*>(vertices));
            } break;

            case Arrow_3D_Style::cube: {
                generate_cube_geometry(reinterpret_cast<Vector3*>(vertices));
            } break;
        }
        return get_required_buffer_size_arrow_3d(style, vertex_count);
    }

    std::optional<float> intersect_arrow_3d(Ray const ray, Arrow_3D const arrow, float const* const world_transform_ptr,
                                            float const* const view_projection_matrix_ptr, uint32_t const viewport_width, uint32_t const viewport_height) {
        Matrix4 const world_transform = Matrix4(world_transform_ptr);
        float const scale = compute_scale(world_transform_ptr, arrow.size, view_projection_matrix_ptr, viewport_width, viewport_height);
        switch (arrow.draw_style) {
            case Arrow_3D_Style::cone: {
                std::optional<float> result = std::nullopt;

                Vector3 const vertex1 = Vector3(Vector4(0, 0, 0, 1) * world_transform);
                Vector3 const vertex2 = Vector3(Vector4(0, 0, -0.8f * scale, 1) * world_transform);
                if (std::optional<Raycast_Hit> const hit = intersect_ray_cylinder(ray, vertex1, vertex2, 0.05f * scale);
                    hit.has_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                Vector3 const vertex = Vector3(Vector4(0.0f, 0.0f, -1.05f, 1.0f) * transform::scale(scale) * world_transform);
                Vector3 const direction = Vector3(Vector4(0.0f, 0.0f, 1.0f, 0.0f) * world_transform);
                // 0.2 height, 0.05 radius
                float const angle_cos = 0.970143f;
                float const height = 0.3f * scale;
                if (std::optional<Raycast_Hit> const hit = intersect_ray_cone(ray, vertex, direction, angle_cos, height);
                    hit.has_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                return result;
            }

            case Arrow_3D_Style::cube: {
                std::optional<float> result = std::nullopt;

                Vector3 const vertex1 = Vector3(Vector4(0, 0, 0, 1) * world_transform);
                Vector3 const vertex2 = Vector3(Vector4(0, 0, -0.8f * scale, 1) * world_transform);
                if (std::optional<Raycast_Hit> const hit = intersect_ray_cylinder(ray, vertex1, vertex2, 0.05f * scale);
                    hit.has_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                OBB cube_bounding_vol;
                cube_bounding_vol.local_x = Vector3(Vector4(1.0f, 0.0f, 0.0f, 0.0f) * world_transform);
                cube_bounding_vol.local_y = Vector3(Vector4(0.0f, 1.0f, 0.0f, 0.0f) * world_transform);
                cube_bounding_vol.local_z = Vector3(Vector4(0.0f, 0.0f, -1.0f, 0.0f) * world_transform);
                cube_bounding_vol.halfwidths = {0.1f * scale, 0.1f * scale, 0.1f * scale};
                cube_bounding_vol.center = transform::get_translation(world_transform) + cube_bounding_vol.local_z * 0.95f * scale;
                if (std::optional<Raycast_Hit> const hit = intersect_ray_obb(ray, cube_bounding_vol); hit.has_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                return result;
            }
        }
    }
} // namespace anton
