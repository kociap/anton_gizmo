#include <anton/gizmo/shapes.hpp>

#include <anton/math/math.hpp>
#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    Array<math::Vec3> generate_filled_circle(f32 const radius, i32 const vertex_count) {
        Array<math::Vec3> circle = generate_circle({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}, radius * 2.0f, vertex_count);
        Array<math::Vec3> result;
        math::Vec3 const v1 = {0.0f, 0.0f, 0.0f};
        for(i64 i = 0; i < vertex_count; ++i) {
            math::Vec3 const& v2 = circle[i];
            math::Vec3 const& v3 = circle[(i + 1) % vertex_count];
            result.emplace_back(v1);
            result.emplace_back(v2);
            result.emplace_back(v3);
        }
        return result;
    }

    Array<math::Vec3> generate_cube(f32 const edge_length) {
        f32 const half_size = 0.5f * edge_length;
        Array<math::Vec3> cube{reserve, 36};
        // Generate cube
        // Top face
        cube.emplace_back(math::Vec3{-half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{-half_size, half_size, -half_size});
        cube.emplace_back(math::Vec3{-half_size, half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, -half_size});
        // Bottom face
        cube.emplace_back(math::Vec3{-half_size, -half_size, half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, -half_size, half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, -half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, -half_size, half_size});
        // Right face
        cube.emplace_back(math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{half_size, -half_size, half_size});
        cube.emplace_back(math::Vec3{half_size, -half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{half_size, -half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, -half_size});
        // Left face
        cube.emplace_back(math::Vec3{-half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, half_size});
        cube.emplace_back(math::Vec3{-half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{-half_size, half_size, -half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, -half_size});
        // Front face
        cube.emplace_back(math::Vec3{-half_size, half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, -half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, -half_size});
        cube.emplace_back(math::Vec3{half_size, -half_size, -half_size});
        // Back face
        cube.emplace_back(math::Vec3{-half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, half_size});
        cube.emplace_back(math::Vec3{-half_size, -half_size, half_size});
        cube.emplace_back(math::Vec3{half_size, -half_size, half_size});
        cube.emplace_back(math::Vec3{half_size, half_size, half_size});
        return cube;
    }

    Array<math::Vec3> generate_icosphere(f32 const radius, i64 const subdivision_level) {
        f32 const vert_inv_len = radius * math::inv_sqrt(1.0f + math::golden_ratio * math::golden_ratio);

        // The first letter is the axis along which the shorter edge of the golden rectangle is
        // The second letter is the axis along which the longer edge of the golden rectangle is
        // Points are in CCW order from the top-left (when plane normal is facing us and the 2nd axis ia oriented upwards).

        math::Vec3 const xy_v1 = math::Vec3{-1.0f, math::golden_ratio, 0.0f} * vert_inv_len;
        math::Vec3 const xy_v2 = math::Vec3{-1.0f, -math::golden_ratio, 0.0f} * vert_inv_len;
        math::Vec3 const xy_v3 = math::Vec3{1.0f, -math::golden_ratio, 0.0f} * vert_inv_len;
        math::Vec3 const xy_v4 = math::Vec3{1.0f, math::golden_ratio, 0.0f} * vert_inv_len;

        math::Vec3 const yz_v1 = math::Vec3{0.0f, -1.0f, math::golden_ratio} * vert_inv_len;
        math::Vec3 const yz_v2 = math::Vec3{0.0f, -1.0f, -math::golden_ratio} * vert_inv_len;
        math::Vec3 const yz_v3 = math::Vec3{0.0f, 1.0f, -math::golden_ratio} * vert_inv_len;
        math::Vec3 const yz_v4 = math::Vec3{0.0f, 1.0f, math::golden_ratio} * vert_inv_len;

        math::Vec3 const zx_v1 = math::Vec3{math::golden_ratio, 0.0f, -1.0f} * vert_inv_len;
        math::Vec3 const zx_v2 = math::Vec3{-math::golden_ratio, 0.0f, -1.0f} * vert_inv_len;
        math::Vec3 const zx_v3 = math::Vec3{-math::golden_ratio, 0.0f, 1.0f} * vert_inv_len;
        math::Vec3 const zx_v4 = math::Vec3{math::golden_ratio, 0.0f, 1.0f} * vert_inv_len;

        Array<math::Vec3> vertices{reserve, 60};

        // Faces around xy_v1 point in CCW order
        vertices.emplace_back(xy_v1);
        vertices.emplace_back(zx_v2);
        vertices.emplace_back(zx_v3);

        vertices.emplace_back(xy_v1);
        vertices.emplace_back(zx_v3);
        vertices.emplace_back(yz_v4);

        vertices.emplace_back(xy_v1);
        vertices.emplace_back(yz_v4);
        vertices.emplace_back(xy_v4);

        vertices.emplace_back(xy_v1);
        vertices.emplace_back(xy_v4);
        vertices.emplace_back(yz_v3);

        vertices.emplace_back(xy_v1);
        vertices.emplace_back(yz_v3);
        vertices.emplace_back(zx_v2);

        // 5 adjacent faces
        vertices.emplace_back(zx_v3);
        vertices.emplace_back(zx_v2);
        vertices.emplace_back(xy_v2);

        vertices.emplace_back(yz_v4);
        vertices.emplace_back(zx_v3);
        vertices.emplace_back(yz_v1);

        vertices.emplace_back(xy_v4);
        vertices.emplace_back(yz_v4);
        vertices.emplace_back(zx_v4);

        vertices.emplace_back(yz_v3);
        vertices.emplace_back(xy_v4);
        vertices.emplace_back(zx_v1);

        vertices.emplace_back(zx_v2);
        vertices.emplace_back(yz_v3);
        vertices.emplace_back(yz_v2);

        // Faces around xy_v3 point in CCW order
        vertices.emplace_back(xy_v3);
        vertices.emplace_back(zx_v1);
        vertices.emplace_back(zx_v4);

        vertices.emplace_back(xy_v3);
        vertices.emplace_back(zx_v4);
        vertices.emplace_back(yz_v1);

        vertices.emplace_back(xy_v3);
        vertices.emplace_back(yz_v1);
        vertices.emplace_back(xy_v2);

        vertices.emplace_back(xy_v3);
        vertices.emplace_back(xy_v2);
        vertices.emplace_back(yz_v2);

        vertices.emplace_back(xy_v3);
        vertices.emplace_back(yz_v2);
        vertices.emplace_back(zx_v1);

        // 5 adjacent faces
        vertices.emplace_back(zx_v4);
        vertices.emplace_back(zx_v1);
        vertices.emplace_back(xy_v4);

        vertices.emplace_back(yz_v1);
        vertices.emplace_back(zx_v4);
        vertices.emplace_back(yz_v4);

        vertices.emplace_back(xy_v2);
        vertices.emplace_back(yz_v1);
        vertices.emplace_back(zx_v3);

        vertices.emplace_back(yz_v2);
        vertices.emplace_back(xy_v2);
        vertices.emplace_back(zx_v2);

        vertices.emplace_back(zx_v1);
        vertices.emplace_back(yz_v2);
        vertices.emplace_back(yz_v3);

        // Subdivision
        for(i64 s = 0; s < subdivision_level; ++s) {
            Array<math::Vec3> subdivided{reserve, 4 * vertices.size()};
            for(i64 i = 0; i < vertices.size(); i += 3) {
                math::Vec3& v1 = vertices[i];
                math::Vec3& v2 = vertices[i + 1];
                math::Vec3& v3 = vertices[i + 2];

                math::Vec3 const a = radius * math::normalize(v1 + v2);
                math::Vec3 const b = radius * math::normalize(v1 + v3);
                math::Vec3 const c = radius * math::normalize(v2 + v3);

                subdivided.emplace_back(v1);
                subdivided.emplace_back(a);
                subdivided.emplace_back(b);

                subdivided.emplace_back(v2);
                subdivided.emplace_back(c);
                subdivided.emplace_back(a);

                subdivided.emplace_back(v3);
                subdivided.emplace_back(b);
                subdivided.emplace_back(c);

                subdivided.emplace_back(a);
                subdivided.emplace_back(c);
                subdivided.emplace_back(b);
            }
            vertices = ANTON_MOV(subdivided);
        }

        return vertices;
    }

    Optional<f32> intersect_cube(math::Ray const ray, f32 const edge_length, math::Mat4 const& world_transform) {
        math::OBB cube_bounding_vol;
        cube_bounding_vol.local_x = math::Vec3(world_transform * math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
        cube_bounding_vol.local_y = math::Vec3(world_transform * math::Vec4(0.0f, 1.0f, 0.0f, 0.0f));
        cube_bounding_vol.local_z = math::Vec3(world_transform * math::Vec4(0.0f, 0.0f, -1.0f, 0.0f));
        cube_bounding_vol.halfwidths = math::Vec3{0.5f * edge_length};
        cube_bounding_vol.center = math::Vec3(world_transform * math::Vec4{0.0f, 0.0f, 0.0f, 1.0f});
        if(Optional<Raycast_Hit> const hit = intersect_ray_obb(ray, cube_bounding_vol)) {
            return Optional<f32>{hit->distance};
        } else {
            return null_optional;
        }
    }

    Optional<f32> intersect_sphere(math::Ray ray, f32 radius, math::Mat4 const& world_transform) {
        math::Vec3 const origin{world_transform * math::Vec4{0.0f, 0.0f, 0.0f, 1.0f}};
        if(Optional<Raycast_Hit> const hit = intersect_ray_sphere(ray, origin, radius)) {
            return Optional<f32>{hit->distance};
        } else {
            return null_optional;
        }
    }
} // namespace anton::gizmo