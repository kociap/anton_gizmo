#include <anton/math/math.hpp>

namespace anton::gizmo {
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
            Array<Vec3> subdivided{reserve, 4 * vertices.size()};
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
            vertices = move(subdivided);
        }

        return vertices;
    }
} // namespace anton::gizmo