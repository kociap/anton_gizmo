#include <anton/gizmo/arrow_3d.hpp>

#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    [[nodiscard]] static Array<math::Vec3> generate_cone_geometry(Arrow_3D const& arrow, i32 const vert_count) {
        Array<math::Vec3> circle = generate_circle(math::Vec3{0.0f}, math::Vec3{0.0f, 0.0f, -1.0f}, 0.5f, vert_count);
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
        Array<math::Vec3> circle = generate_circle(math::Vec3{0.0f}, math::Vec3{0.0f, 0.0f, -1.0f}, 0.5f, vert_count);
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
            case Arrow_3D_Style::cone:
                return generate_cone_geometry(arrow, vertex_count);

            case Arrow_3D_Style::cube:
                return generate_cube_geometry(arrow, vertex_count);
        }
    }

    math::Mat4 calculate_transform(math::Mat4 const& world_transform, f32 const fixed_scale_factor) {
        math::Vec3 const z_axis = calculate_world_direction(world_transform, {0.0f, 0.0f, -1.0f});
        math::Vec3 y_axis = calculate_world_direction(world_transform, {0.0f, 1.0f, 0.0f});
        // Orthonormalize the y axis
        y_axis -= dot(z_axis, y_axis) * z_axis;
        y_axis = normalize(y_axis);
        math::Vec3 const x_axis = cross(z_axis, y_axis);
        math::Vec4 const x_basis{fixed_scale_factor * x_axis, 0.0f};
        math::Vec4 const y_basis{fixed_scale_factor * y_axis, 0.0f};
        math::Vec4 const z_basis{fixed_scale_factor * z_axis, 0.0f};
        math::Vec4 const origin{calculate_world_origin(world_transform), 1.0f};
        math::Mat4 const transform{x_basis, y_basis, z_basis, origin};
        return transform;
    }

    Optional<f32> intersect_arrow_3d(math::Ray const ray, Arrow_3D const& arrow, math::Mat4 const& gizmo_transform) {
        Optional<f32> result = null_optional;
        // Uniformly scaled - all axes have the same scale applied
        f32 const scale = math::length(gizmo_transform[0]);
        math::Vec3 const origin{gizmo_transform * math::Vec4{0.0f, 0.0f, 0.0f, 1.0f}};
        math::Vec3 const direction_scaled{gizmo_transform * math::Vec4{0.0f, 0.0f, -1.0f, 0.0f}};
        math::Vec3 const direction = normalize(direction_scaled);
        f32 const shaft_radius = 0.5f * scale * arrow.shaft_diameter;
        f32 const shaft_length = scale * arrow.shaft_length;
        Optional<Raycast_Hit> const shaft_hit = intersect_ray_cylinder(ray, origin, origin + direction * shaft_length, shaft_radius);
        if(shaft_hit) {
            result = shaft_hit->distance;
        }

        switch(arrow.draw_style) {
            case Arrow_3D_Style::cone: {
                math::Vec3 const cone_origin = origin + scale * (arrow.shaft_length + arrow.cap_length) * direction;
                math::Vec3 const cone_direction = -direction;
                f32 const cone_height = arrow.cap_length;
                f32 const cone_radius = 0.5f * arrow.cap_size;
                // cos(a) = adjacent / hypotenuse
                f32 const angle_cos = cone_height * math::inv_sqrt(cone_height * cone_height + cone_radius * cone_radius);
                Optional<Raycast_Hit> const cone_hit = intersect_ray_cone(ray, cone_origin, cone_direction, angle_cos, scale * cone_height);
                if(cone_hit && (!result || cone_hit->distance < *result)) {
                    result = cone_hit->distance;
                }
            } break;

            case Arrow_3D_Style::cube: {
                math::Vec3 const x_axis{gizmo_transform * math::Vec4{1.0f, 0.0f, 0.0f, 0.0f}};
                math::Vec3 const y_axis{gizmo_transform * math::Vec4{0.0f, 1.0f, 0.0f, 0.0f}};
                math::Vec3 const z_axis{direction};
                math::OBB cube_bounding_vol;
                cube_bounding_vol.local_x = x_axis;
                cube_bounding_vol.local_y = y_axis;
                cube_bounding_vol.local_z = z_axis;
                cube_bounding_vol.halfwidths = math::Vec3{0.5f * scale * arrow.cap_size};
                cube_bounding_vol.center = origin + scale * (arrow.shaft_length - 0.5f * arrow.cap_size) * direction;
                Optional<Raycast_Hit> const cube_hit = intersect_ray_obb(ray, cube_bounding_vol);
                if(cube_hit && (!result || cube_hit->distance < *result)) {
                    result = cube_hit->distance;
                }
            } break;
        }

        return result;
    }
} // namespace anton::gizmo
