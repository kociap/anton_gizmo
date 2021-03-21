#include <anton/gizmo/shapes.hpp>

#include <anton/math/math.hpp>
#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    Array<math::Vec3> generate_filled_circle(i32 const vertex_count) {
        math::Vec3 const origin{0.0f, 0.0f, 0.0f};
        Array<math::Vec3> circle = generate_circle(origin, math::Vec3{0.0f, 0.0f, -1.0f}, 1.0f, vertex_count);
        Array<math::Vec3> result{reserve, 3 * vertex_count};
        for(i64 i = 0; i < vertex_count; ++i) {
            math::Vec3 const& v2 = circle[i];
            math::Vec3 const& v3 = circle[(i + 1) % vertex_count];
            result.emplace_back(origin);
            result.emplace_back(v3);
            result.emplace_back(v2);
        }
        return result;
    }

    Array<math::Vec3> generate_square() {
        math::Vec3 const v1{0.5f, 0.5f, 0.0f};
        math::Vec3 const v2{0.5f, -0.5f, 0.0f};
        math::Vec3 const v3{-0.5f, 0.5f, 0.0f};
        math::Vec3 const v4{-0.5f, -0.5f, 0.0f};
        Array<math::Vec3> square{reserve, 6};
        square.emplace_back(v1);
        square.emplace_back(v2);
        square.emplace_back(v3);
        square.emplace_back(v3);
        square.emplace_back(v2);
        square.emplace_back(v4);
        return square;
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

    Array<math::Vec3> generate_icosphere(i64 const subdivision_level) {
        // The vertices of an icoshpere are formed by the corners of 3 orthogonal intersecting
        // rectangles with edge lengths 1 and golden ratio.
        // The first letter is the axis along which the shorter edge of the rectangle is.
        // The second letter is the axis along which the longer edge of the rectangle is.
        // Points are in CCW order from the top-left (when the plane normal is facing us and the 2nd axis is oriented upwards).

        f32 const vert_inv_len = math::inv_sqrt(1.0f + math::golden_ratio * math::golden_ratio);

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

                math::Vec3 const a = math::normalize(v1 + v2);
                math::Vec3 const b = math::normalize(v1 + v3);
                math::Vec3 const c = math::normalize(v2 + v3);

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

    Optional<f32> intersect_circle(math::Ray const& ray, math::Mat4 const& world_transform) {
        math::Vec3 const world_origin{world_transform * math::Vec4{0.0f, 0.0f, 0.0f, 1.0f}};
        math::Mat4 const inverse_transform{math::inverse(world_transform)};
        math::Mat4 const transpose_inverse_transform{math::transpose(inverse_transform)};
        math::Vec3 const world_normal{math::normalize(transpose_inverse_transform * math::Vec4{0.0f, 0.0f, -1.0f, 0.0f})};
        f32 const plane_distance = math::dot(world_origin, world_normal);
        Optional<Raycast_Hit> const hit = intersect_ray_plane(ray, world_normal, plane_distance);
        if(!hit) {
            return null_optional;
        }

        math::Vec4 const radius_vector{1.0f, 0.0f, 0.0f, 0.0f};
        math::Vec3 const world_radius{world_transform * radius_vector};
        f32 const radius_squared = math::length_squared(world_radius);
        f32 const hit_distance_squared = math::length_squared(hit->hit_point - world_origin);
        if(hit_distance_squared <= radius_squared) {
            return hit->distance;
        } else {
            return null_optional;
        }
    }

    Optional<f32> intersect_square(math::Ray const& ray, math::Mat4 const& world_transform) {
        math::Vec3 const world_origin{world_transform * math::Vec4{0.0f, 0.0f, 0.0f, 1.0f}};
        math::Mat4 const inverse_transform{math::inverse(world_transform)};
        math::Mat4 const transpose_inverse_transform{math::transpose(inverse_transform)};
        math::Vec3 const world_normal{math::normalize(transpose_inverse_transform * math::Vec4{0.0f, 0.0f, -1.0f, 0.0f})};
        f32 const plane_distance = math::dot(world_origin, world_normal);
        Optional<Raycast_Hit> const hit = intersect_ray_plane(ray, world_normal, plane_distance);
        if(!hit) {
            return null_optional;
        }

        math::Vec3 const p{hit->hit_point - world_origin};
        math::Vec3 world_u{world_transform * math::Vec4{0.0f, 1.0f, 0.0f, 0.0f}};
        f32 const length_u = math::length(world_u);
        world_u /= length_u;
        f32 const d_u = math::abs(math::dot(p, world_u));
        math::Vec3 world_r{world_transform * math::Vec4{1.0f, 0.0f, 0.0f, 0.0f}};
        f32 const length_r = math::length(world_r);
        world_r /= length_r;
        f32 const d_r = math::abs(math::dot(p, world_r));
        if(d_u <= 0.5f * length_u && d_r <= 0.5f * length_r) {
            return hit->distance;
        } else {
            return null_optional;
        }
    }

    Optional<f32> intersect_cube(math::Ray const& ray, math::Mat4 const& world_transform) {
        math::OBB cube_bounding_vol;
        cube_bounding_vol.local_x = math::normalize(math::Vec3(world_transform * math::Vec4(1.0f, 0.0f, 0.0f, 0.0f)));
        cube_bounding_vol.local_y = math::normalize(math::Vec3(world_transform * math::Vec4(0.0f, 1.0f, 0.0f, 0.0f)));
        cube_bounding_vol.local_z = math::normalize(math::Vec3(world_transform * math::Vec4(0.0f, 0.0f, -1.0f, 0.0f)));
        cube_bounding_vol.halfwidths = math::Vec3{0.5f * math::length(math::Vec3{world_transform[0]})};
        cube_bounding_vol.center = math::Vec3(world_transform * math::Vec4{0.0f, 0.0f, 0.0f, 1.0f});
        Optional<Raycast_Hit> const hit = intersect_ray_obb(ray, cube_bounding_vol);
        if(hit) {
            return hit->distance;
        } else {
            return null_optional;
        }
    }

    Optional<f32> intersect_sphere(math::Ray const& ray, math::Mat4 const& world_transform) {
        math::Vec3 const origin{world_transform * math::Vec4{0.0f, 0.0f, 0.0f, 1.0f}};
        f32 const radius = math::length(math::Vec3{world_transform[0]});
        Optional<Raycast_Hit> const hit = intersect_ray_sphere(ray, origin, radius);
        if(hit) {
            return hit->distance;
        } else {
            return null_optional;
        }
    }
} // namespace anton::gizmo
