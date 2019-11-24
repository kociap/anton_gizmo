#include <anton_gizmo/dial_3d.hpp>

#include <anton_gizmo/utils.hpp>
#include <intersection_tests.hpp>

namespace anton {
    uint32_t get_required_buffer_size_dial_3d(uint32_t vertex_count) {
        return 2 * vertex_count + 2;
    }

    uint32_t generate_dial_3d_geometry(uint32_t const vertex_count, Vector3* const vertices, Vector3* const rotation_axes, float* const scale_factors) {
        float const angle = constants::pi / vertex_count;
        for (int32_t i = 0; i <= 2 * vertex_count; i += 2) {
            Vector3 const pos = {cos(angle * i), sin(angle * i), 0};
            vertices[i] = pos;
            vertices[i + 1] = pos;
        }

        for (int32_t i = 2; i <= 2 * vertex_count - 2; i += 2) {
            // normal = (vert[i] - vert[i - 2]) + (vert[i + 2] - vert[i])
            Vector3 const normal = normalize(vertices[i + 2] - vertices[i - 2]);
            rotation_axes[i] = normal;
            rotation_axes[i + 1] = normal;

            Vector3 const tangent = Vector3{-normal.y, normal.x, 0};
            Vector3 const line_segment = normalize(vertices[i] - vertices[i - 2]);
            float const factor = 1.0f / abs(dot(Vector3{-line_segment.y, line_segment.x, 0}, tangent));
            scale_factors[i] = factor;
            scale_factors[i + 1] = factor;
        }

        // First and last vertices are duplicates
        Vector3 const normal = normalize(vertices[2] - vertices[2 * vertex_count - 2]);
        rotation_axes[0] = normal;
        rotation_axes[1] = normal;
        rotation_axes[2 * vertex_count] = normal;
        rotation_axes[2 * vertex_count + 1] = normal;

        Vector3 const line_segment = normalize(vertices[0] - vertices[2 * vertex_count - 2]);
        Vector3 const tangent = Vector3{-normal.y, normal.x, 0};
        float const factor = 1 / abs(dot(Vector3{-line_segment.y, line_segment.x, 0}, tangent));
        scale_factors[0] = factor;
        scale_factors[1] = factor;
        scale_factors[2 * vertex_count] = factor;
        scale_factors[2 * vertex_count + 1] = factor;

        return 2 * vertex_count + 2;
    }

    std::optional<float> intersect_dial_3d(Ray, Dial_3D, Matrix4 world_transform, Matrix4 view_projection_matrix, Vector2 viewport_size) {
        return std::nullopt;
    }
} // namespace anton
