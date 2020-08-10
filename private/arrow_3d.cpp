#include <anton/gizmo/arrow_3d.hpp>

#include <anton/math/mat4.hpp>
#include <anton/math/quat.hpp>
#include <anton/math/vec2.hpp>
#include <anton/math/vec3.hpp>
#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    [[nodiscard]] static Array<math::Vec3> generate_cone_geometry(Arrow_3D const& arrow, i32 const vert_count) {
        Array<math::Vec3> circle = generate_circle(math::Vec3{0.0f}, math::Vec3{0.0f, 0.0f, -1.0f}, 1.0f, vert_count);
        f32 const cap_size = arrow.cap_size;
        f32 const cap_length = arrow.cap_length;
        f32 const shaft_length = arrow.shaft_length;
        f32 const shaft_diameter = arrow.shaft_diameter;
        Array<math::Vec3> cone{reserve, (i64)vert_count * (3 + 3 + 6 + 3)};
        for(i64 i = 0; i < vert_count; ++i) {
            math::Vec3& v1 = circle[i];
            math::Vec3& v2 = circle[(i + 1) % vert_count];
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

    [[nodiscard]] static Array<math::Vec3> generate_cube_geometry(Arrow_3D const& arrow, i32 const vert_count) {
        Array<math::Vec3> circle = generate_circle(math::Vec3{0.0f}, math::Vec3{0.0f, 0.0f, -1.0f}, 1.0f, vert_count);
        f32 const shaft_length = arrow.shaft_length;
        f32 const shaft_diameter = arrow.shaft_diameter;
        f32 const half_size = 0.5f * arrow.cap_size;
        math::Vec3 const offset = {0, 0, -shaft_length + half_size};
        Array<math::Vec3> cube{reserve, 36 + (3 + 6 + 3) * (i64)vert_count};
        // Generate cube
        // Top face
        cube.emplace_back(offset + math::Vec3{-half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, -half_size});
        // Bottom face
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, -half_size, half_size});
        // Right face
        cube.emplace_back(offset + math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vec3{half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, -half_size});
        // Left face
        cube.emplace_back(offset + math::Vec3{-half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, -half_size});
        // Front face
        cube.emplace_back(offset + math::Vec3{-half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, -half_size});
        cube.emplace_back(offset + math::Vec3{half_size, -half_size, -half_size});
        // Back face
        cube.emplace_back(offset + math::Vec3{-half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(offset + math::Vec3{-half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vec3{half_size, -half_size, half_size});
        cube.emplace_back(offset + math::Vec3{half_size, half_size, half_size});
        // Generate shaft
        for(i64 i = 0; i < vert_count; ++i) {
            math::Vec3& v1 = circle[i];
            math::Vec3& v2 = circle[(i + 1) % vert_count];
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

    anton::Array<math::Vec3> generate_arrow_3d_geometry(Arrow_3D const& arrow, i32 const vertex_count) {
        switch(arrow.draw_style) {
            case Arrow_3D_Style::cone: {
                return generate_cone_geometry(arrow, vertex_count);
            } break;

            case Arrow_3D_Style::cube: {
                return generate_cube_geometry(arrow, vertex_count);
            } break;
        }
    }

    Optional<f32> intersect_arrow_3d(math::Ray const ray, Arrow_3D const& arrow, math::Mat4 const& world_transform) {
        switch(arrow.draw_style) {
            case Arrow_3D_Style::cone: {
                Optional<f32> result = null_optional;
                math::Vec3 const vertex1 = math::Vec3(world_transform * math::Vec4(0, 0, 0, 1));
                math::Vec3 const vertex2 = math::Vec3(world_transform * math::Vec4(0, 0, -arrow.shaft_length, 1));
                f32 const radius = 0.5f * arrow.shaft_diameter;
                if(Optional<Raycast_Hit> const hit = intersect_ray_cylinder(ray, vertex1, vertex2, radius);
                   hit.holds_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                math::Vec3 const vertex = math::Vec3(world_transform * math::Vec4(0.0f, 0.0f, -arrow.shaft_length - arrow.cap_length, 1.0f));
                math::Vec3 const direction = math::Vec3(world_transform * math::Vec4(0.0f, 0.0f, 1.0f, 0.0f));
                f32 const cone_height = arrow.cap_length;
                f32 const cone_radius = 0.5f * arrow.cap_size;
                f32 const angle_cos = cone_height * math::inv_sqrt(cone_height * cone_height + cone_radius * cone_radius);
                if(Optional<Raycast_Hit> const hit = intersect_ray_cone(ray, vertex, direction, angle_cos, cone_height);
                   hit.holds_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                return result;
            }

            case Arrow_3D_Style::cube: {
                Optional<f32> result = null_optional;
                math::Vec3 const vertex1 = math::Vec3(world_transform * math::Vec4(0, 0, 0, 1));
                math::Vec3 const vertex2 = math::Vec3(world_transform * math::Vec4(0, 0, -arrow.shaft_length, 1));
                f32 const radius = 0.5f * arrow.shaft_diameter;
                if(Optional<Raycast_Hit> const hit = intersect_ray_cylinder(ray, vertex1, vertex2, radius);
                   hit.holds_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                math::OBB cube_bounding_vol;
                cube_bounding_vol.local_x = math::Vec3(world_transform * math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
                cube_bounding_vol.local_y = math::Vec3(world_transform * math::Vec4(0.0f, 1.0f, 0.0f, 0.0f));
                cube_bounding_vol.local_z = math::Vec3(world_transform * math::Vec4(0.0f, 0.0f, -1.0f, 0.0f));
                cube_bounding_vol.halfwidths = math::Vec3{0.5f * arrow.cap_size};
                cube_bounding_vol.center = math::Vec3(world_transform * math::Vec4{0.0f, 0.0f, (-arrow.shaft_length + 0.5f * arrow.cap_size), 1.0f});
                if(Optional<Raycast_Hit> const hit = intersect_ray_obb(ray, cube_bounding_vol); hit.holds_value() && (!result || hit->distance < *result)) {
                    result = hit->distance;
                }

                return result;
            }
        }
    }
} // namespace anton::gizmo
