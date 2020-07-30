#include <anton/gizmo/dial_3d.hpp>

#include <anton/math/matrix4.hpp>
#include <anton/math/vector2.hpp>
#include <anton/math/vector3.hpp>
#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    u32 get_required_buffer_size_dial_3d(u32 vertex_count) {
        return 0;
    }

    void generate_dial_3d_geometry(u32 const vertex_count, Slice<math::Vector3> const vertices) {
        // f32 const angle = math::pi / vertex_count;
        // for (u64 i = 0; i <= 2 * u64(vertex_count); i += 2) {
        //     math::Vector3 const pos = {cos(angle * i), sin(angle * i), 0};
        //     vertices[i] = pos;
        //     vertices[i + 1] = pos;
        // }

        // for (u64 i = 2; i <= 2 * u64(vertex_count) - 2; i += 2) {
        //     // normal = (vert[i] - vert[i - 2]) + (vert[i + 2] - vert[i])
        //     math::Vector3 const normal = normalize(vertices[i + 2] - vertices[i - 2]);
        //     rotation_axes[i] = normal;
        //     rotation_axes[i + 1] = normal;

        //     math::Vector3 const tangent = math::Vector3{-normal.y, normal.x, 0};
        //     math::Vector3 const line_segment = normalize(vertices[i] - vertices[i - 2]);
        //     f32 const factor = 1.0f / abs(dot(math::Vector3{-line_segment.y, line_segment.x, 0}, tangent));
        //     scale_factors[i] = factor;
        //     scale_factors[i + 1] = factor;
        // }

        // // First and last vertices are duplicates
        // math::Vector3 const normal = normalize(vertices[2] - vertices[2 * vertex_count - 2]);
        // rotation_axes[0] = normal;
        // rotation_axes[1] = normal;
        // rotation_axes[2 * vertex_count] = normal;
        // rotation_axes[2 * vertex_count + 1] = normal;

        // math::Vector3 const line_segment = normalize(vertices[0] - vertices[2 * vertex_count - 2]);
        // math::Vector3 const tangent = math::Vector3{-normal.y, normal.x, 0};
        // f32 const factor = 1 / abs(dot(math::Vector3{-line_segment.y, line_segment.x, 0}, tangent));
        // scale_factors[0] = factor;
        // scale_factors[1] = factor;
        // scale_factors[2 * vertex_count] = factor;
        // scale_factors[2 * vertex_count + 1] = factor;
    }

    Optional<f32> intersect_dial_3d(Ray, Dial_3D, f32 const* const, f32 const* const, math::Vector2 const) {
        return null_optional;
    }
} // namespace anton::gizmo
