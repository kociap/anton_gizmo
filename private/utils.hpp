#pragma once

#include <anton/array.hpp>
#include <anton/math/mat4.hpp>
#include <anton/math/primitives.hpp>
#include <anton/math/quat.hpp>
#include <anton/math/vec3.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    // generate_circle
    // Generate a circle of diameter in the plane n = normal, d = dot(origin, normal) centered at origin.
    // The vertices are generated in clockwise order.
    //
    [[maybe_unused]] [[nodiscard]] static anton::Array<math::Vec3> generate_circle(math::Vec3 const& origin, math::Vec3 const& normal, f32 const radius,
                                                                                   i32 const vert_count) {
        f32 const angle = math::two_pi / static_cast<f32>(vert_count);
        math::Quat const rotation_quat = math::Quat::from_axis_angle(normal, angle);
        // Find a point in the plane n = normal, d = 0
        math::Vec3 vertex = math::perpendicular(normal);
        // Rescale the circle
        vertex *= radius;
        // Generate a circle in the plane n = normal, d = 0 and center it at origin
        math::Quat rotated_vec{vertex.x, vertex.y, vertex.z, 0.0f};
        anton::Array<math::Vec3> circle_points{anton::reserve, vert_count};
        for(i64 i = 0; i <= vert_count; ++i) {
            rotated_vec = rotation_quat * rotated_vec * conjugate(rotation_quat);
            circle_points.emplace_back(rotated_vec.x + origin.x, rotated_vec.y + origin.y, rotated_vec.z + origin.z);
        }
        return circle_points;
    }

    [[maybe_unused]] [[nodiscard]] static math::Vec3 calculate_world_origin(math::Mat4 const& world_transform) {
        math::Vec3 const origin{world_transform * math::Vec4{0.0f, 0.0f, 0.0f, 1.0f}};
        return origin;
    }

    [[maybe_unused]] [[nodiscard]] static math::Vec3 calculate_world_direction(math::Mat4 const& world_transform, math::Vec3 const vector) {
        math::Vec3 const world_direciton{world_transform * math::Vec4{vector, 0.0f}};
        math::Vec3 const direction = math::normalize(world_direciton);
        return direction;
    }
} // namespace anton::gizmo
