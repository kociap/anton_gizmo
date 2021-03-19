#include <anton/gizmo/dial_3d.hpp>

#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    Array<math::Vec3> generate_dial_3d_geometry(Dial_3D const& dial, i32 const vertex_count_major, i32 const vertex_count_minor) {
        Array<math::Vec3> major = generate_circle(math::Vec3{0.0f}, math::Vec3{0.0f, 0.0f, -1.0f}, dial.major_radius, vertex_count_major);
        Array<Array<math::Vec3>> torus_rings{reserve, vertex_count_major};
        for(i64 i = 0; i < vertex_count_major; ++i) {
            math::Vec3 const& v1 = major[i];
            math::Vec3 const& v2 = major[(i + 1) % vertex_count_major];
            math::Vec3 const plane_normal = math::normalize(v1 - v2);
            Array<math::Vec3> ring = generate_circle(v2, plane_normal, dial.minor_radius, vertex_count_minor);
            torus_rings.emplace_back(ANTON_MOV(ring));
        }

        Array<math::Vec3> vertices;
        for(i64 i = 0; i < vertex_count_major; ++i) {
            Array<math::Vec3> const& r1 = torus_rings[i];
            Array<math::Vec3> const& r2 = torus_rings[(i + 1) % vertex_count_major];
            for(i64 j = 0; j < vertex_count_minor; ++j) {
                math::Vec3 const& r1_v1 = r1[j];
                math::Vec3 const& r1_v2 = r1[(j + 1) % vertex_count_minor];
                math::Vec3 const& r2_v1 = r2[j];
                math::Vec3 const& r2_v2 = r2[(j + 1) % vertex_count_minor];
                // 1st triangle
                vertices.emplace_back(r2_v1);
                vertices.emplace_back(r2_v2);
                vertices.emplace_back(r1_v2);
                // 2nd triangle
                vertices.emplace_back(r1_v1);
                vertices.emplace_back(r2_v1);
                vertices.emplace_back(r1_v2);
            }
        }
        return vertices;
    }

    Optional<f32> intersect_dial_3d(math::Ray const ray, Dial_3D const& dial, math::Mat4 const& world_transform) {
        math::Vec3 const origin{world_transform * math::Vec4{0.0f}};
        math::Vec3 const v1{world_transform * math::Vec4{0.0f, 0.0f, -dial.minor_radius, 1.0f}};
        math::Vec3 const v2{world_transform * math::Vec4{0.0f, 0.0f, dial.minor_radius, 1.0f}};
        f32 const scale = math::length(v2 - v1) / (2 * dial.minor_radius);
        f32 const r_large = scale * (dial.major_radius + dial.minor_radius);
        f32 const r_small = scale * (dial.major_radius - dial.minor_radius);
        Optional<f32> result = null_optional;
        Optional<Raycast_Hit> const large_hit = intersect_ray_cylinder(ray, v1, v2, r_large);
        if(large_hit) {
            result = large_hit->distance;
        }

        if(!math::is_almost_zero(r_small, 0.001f)) {
            // We create a cutout by testing for an intersection with a cylinder located inside the larger cylinder.
            Optional<Raycast_Hit> const cutout_hit = intersect_ray_cylinder(ray, v1, v2, r_small);
            if(cutout_hit && (result && cutout_hit->distance <= *result)) {
                result = null_optional;
            }

            // We have to test for the ring as well to make sure that the dial can be hit from the inside.
            Optional<Raycast_Hit> const ring_hit = intersect_ray_cylinder_uncapped(ray, v1, v2, r_small);
            if(ring_hit && (!result || ring_hit->distance < *result)) {
                result = ring_hit->distance;
            }
        }

        return result;
    }
} // namespace anton::gizmo
