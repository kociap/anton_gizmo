#include <anton/gizmo/arrow_3d.hpp>

#include <anton/array.hpp>
#include <anton/math/matrix4.hpp>
#include <anton/math/quaternion.hpp>
#include <anton/math/vector2.hpp>
#include <anton/math/vector3.hpp>
#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    [[nodiscard]] static anton::Array<math::Vector3> generate_cone_geometry(Arrow_3D const& arrow, i32 const vert_count) {
        anton::Array<math::Vector3> circle = generate_circle(vert_count);
        f32 const cap_size = arrow.cap_size;
        f32 const cap_length = arrow.cap_length;
        f32 const shaft_length = arrow.shaft_length;
        f32 const shaft_diameter = arrow.shaft_diameter;
        anton::Array<math::Vector3> cone{anton::reserve, (i64)vert_count * (3 + 3 + 6 + 3)};
        for (i64 i = 0; i < vert_count; ++i) {
            math::Vector3& v1 = circle[i];
            math::Vector3& v2 = circle[(i + 1) % vert_count];
            // Cone
            cone.emplace_back(v1.x * cap_size, v1.y * cap_size, -shaft_length);
            cone.emplace_back(v2.x * cap_size, v2.y * cap_size, -shaft_length);
            cone.emplace_back(0.0f, 0.0f, -shaft_length - cap_length);
            // Cone base
            cone.emplace_back(0.0f, 0.0f, -shaft_length);
            cone.emplace_back(v2.x * cap_size, v2.y * cap_size, -shaft_length);
            cone.emplace_back(v1.x * cap_size, v1.y * cap_size, -shaft_length);
            // We don't generate 1st cylinder cap
            // Cylinder
            cone.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, -shaft_length);
            cone.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, 0.0f);
            cone.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, 0.0f);
            cone.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, 0.0f);
            cone.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, -shaft_length);
            cone.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, -shaft_length);
            // 2nd cylinder cap
            cone.emplace_back(0.0f, 0.0f, 0.0f);
            cone.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, 0.0f);
            cone.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, 0.0f);
        }
        return cone;
    }

    [[nodiscard]] static anton::Array<math::Vector3> generate_cube_geometry(Arrow_3D const& arrow, i32 const vert_count) {
        anton::Array<math::Vector3> circle = generate_circle(vert_count);
        f32 const shaft_length = math::clamp(arrow.shaft_length, 0.0f, 1.0f);
        f32 const cap_size = arrow.cap_size;
        f32 const shaft_diameter = arrow.shaft_diameter;
        f32 const half_size = cap_size / 2.0f;
        math::Vector3 const offset = {0, 0, -shaft_length + half_size};
        anton::Array<math::Vector3> cube{anton::reserve, (i64)vert_count * (3 + 3 + 6 + 3)};
        // Generate cube
        cube.emplace_back(offset + math::Vector3{half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vector3{-half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vector3{half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vector3{-half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vector3{-half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vector3{-half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vector3{-half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vector3{half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vector3{half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vector3{half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vector3{half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vector3{-half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vector3{half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vector3{-half_size, half_size, half_size});
        // Generate shaft
        for (i64 i = 0; i < vert_count; ++i) {
            math::Vector3& v1 = circle[i];
            math::Vector3& v2 = circle[(i + 1) % vert_count];
            // 1st cylinder cap
            cube.emplace_back(0.0f, 0.0f, 0.0f);
            cube.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, 0.0f);
            cube.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, 0.0f);
            // Cylinder
            cube.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, -shaft_length);
            cube.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, 0.0f);
            cube.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, 0.0f);
            cube.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, 0.0f);
            cube.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, -shaft_length);
            cube.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, -shaft_length);
            // 2nd cylinder cap
            cube.emplace_back(0.0f, 0.0f, 0.0f);
            cube.emplace_back(v2.x * shaft_diameter, v2.y * shaft_diameter, -shaft_length);
            cube.emplace_back(v1.x * shaft_diameter, v1.y * shaft_diameter, -shaft_length);
        }
        return cube;
    }

    anton::Array<math::Vector3> generate_arrow_3d_geometry(Arrow_3D const& arrow, u32 const vertex_count) {
        switch (arrow.draw_style) {
            case Arrow_3D_Style::cone: {
                return generate_cone_geometry(arrow, vertex_count);
            } break;

            case Arrow_3D_Style::cube: {
                return generate_cube_geometry(arrow, vertex_count);
            } break;
        }
    }

    Optional<f32> intersect_arrow_3d(Ray const ray, Arrow_3D const& arrow, math::Matrix4 const& world_transform, math::Matrix4 const& view_projection,
                                     math::Vector2 const viewport_size) {
        f32 const scale = compute_scale(world_transform, arrow.size, view_projection, viewport_size);
        switch (arrow.draw_style) {
            case Arrow_3D_Style::cone: {
                Optional<f32> result = null_optional;

                math::Vector3 const vertex1 = math::Vector3(world_transform * math::Vector4(0, 0, 0, 1));
                math::Vector3 const vertex2 = math::Vector3(world_transform * math::Vector4(0, 0, -arrow.shaft_length * scale, 1));
                f32 const radius = 0.5f * arrow.shaft_diameter * scale;
                if (Optional<Raycast_Hit> const hit = intersect_ray_cylinder(ray, vertex1, vertex2, radius);
                    hit.holds_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                math::Vector3 const vertex =
                    math::Vector3(world_transform * math::scale(scale) * math::Vector4(0.0f, 0.0f, -arrow.shaft_length - arrow.cap_length, 1.0f));
                math::Vector3 const direction = math::Vector3(world_transform * math::Vector4(0.0f, 0.0f, 1.0f, 0.0f));
                f32 const angle_cos = arrow.cap_length * math::inv_sqrt(arrow.cap_length * arrow.cap_length + arrow.cap_size * arrow.cap_size);
                f32 const height = arrow.cap_length * scale;
                if (Optional<Raycast_Hit> const hit = intersect_ray_cone(ray, vertex, direction, angle_cos, height);
                    hit.holds_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                return result;
            }

            case Arrow_3D_Style::cube: {
                Optional<f32> result = null_optional;

                math::Vector3 const vertex1 = math::Vector3(world_transform * math::Vector4(0, 0, 0, 1));
                math::Vector3 const vertex2 = math::Vector3(world_transform * math::Vector4(0, 0, -arrow.shaft_length * scale, 1));
                f32 const radius = 0.5f * arrow.shaft_diameter * scale;
                if (Optional<Raycast_Hit> const hit = intersect_ray_cylinder(ray, vertex1, vertex2, radius);
                    hit.holds_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                OBB cube_bounding_vol;
                cube_bounding_vol.local_x = math::Vector3(world_transform * math::Vector4(1.0f, 0.0f, 0.0f, 0.0f));
                cube_bounding_vol.local_y = math::Vector3(world_transform * math::Vector4(0.0f, 1.0f, 0.0f, 0.0f));
                cube_bounding_vol.local_z = math::Vector3(world_transform * math::Vector4(0.0f, 0.0f, -1.0f, 0.0f));
                cube_bounding_vol.halfwidths = math::Vector3{0.5f * arrow.cap_size * scale};
                cube_bounding_vol.center = math::Vector3(world_transform * math::Vector4(0.0f, 0.0f, (-arrow.shaft_length + 0.5f * arrow.cap_size) * scale));
                if (Optional<Raycast_Hit> const hit = intersect_ray_obb(ray, cube_bounding_vol); hit.holds_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                return result;
            }
        }
    }
} // namespace anton::gizmo
