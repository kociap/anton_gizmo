#include <anton/gizmo/dial_3d.hpp>

#include <anton/math/matrix4.hpp>
#include <anton/math/quaternion.hpp>
#include <anton/math/vector2.hpp>
#include <anton/math/vector3.hpp>
#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    Array<math::Vector3> generate_dial_3d_geometry(Dial_3D const& dial, i32 const vertex_count_major, i32 const vertex_count_minor) {
        Array<math::Vector3> major = generate_circle(math::Vector3{0.0f}, math::Vector3{0.0f, 0.0f, -1.0f}, dial.major_diameter, vertex_count_major);
        Array<Array<math::Vector3>> torus_rings{reserve, vertex_count_major};
        for(i64 i = 0; i < vertex_count_major; ++i) {
            math::Vector3 const& v1 = major[i];
            math::Vector3 const& v2 = major[(i + 1) % vertex_count_major];
            math::Vector3 const plane_normal = math::normalize(v1 - v2);
            Array<math::Vector3> ring = generate_circle(v2, plane_normal, dial.minor_diameter, vertex_count_minor);
            torus_rings.emplace_back(move(ring));
        }

        Array<math::Vector3> vertices;
        for(i64 i = 0; i < vertex_count_major; ++i) {
            Array<math::Vector3> const& r1 = torus_rings[i];
            Array<math::Vector3> const& r2 = torus_rings[(i + 1) % vertex_count_major];
            for(i64 j = 0; j < vertex_count_minor; ++j) {
                math::Vector3 const& r1_v1 = r1[j];
                math::Vector3 const& r1_v2 = r1[(j + 1) % vertex_count_minor];
                math::Vector3 const& r2_v1 = r2[j];
                math::Vector3 const& r2_v2 = r2[(j + 1) % vertex_count_minor];
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

    Optional<f32> intersect_dial_3d(Ray const ray, Dial_3D const& dial, math::Matrix4 const& world_transform) {
        return null_optional;
    }
} // namespace anton::gizmo
