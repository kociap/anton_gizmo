#pragma once

#include <anton/array.hpp>
#include <anton/gizmo/common.hpp>
#include <anton/math/quaternion.hpp>

namespace anton::gizmo {
    // generate_circle
    // Generate a circle of diameter in the plane n = normal, d = dot(origin, normal) centered at origin.
    // The vertices are generated in clockwise order.
    //
    inline anton::Array<math::Vector3> generate_circle(math::Vector3 const origin, math::Vector3 const normal, f32 const diameter, i32 const vert_count) {
        f32 const angle = math::radians(360.0f / static_cast<f32>(vert_count));
        math::Vector3 const rotation_axis = normal * math::sin(angle / 2.0f);
        math::Quaternion const rotation_quat(rotation_axis.x, rotation_axis.y, rotation_axis.z, math::cos(angle / 2.0f));
        // Find a point in the plane n = normal, d = 0
        math::Vector3 vertex;
        if(normal.x == 0.0f) {
            vertex = math::normalize(math::Vector3{0.0f, -normal.z, normal.y});
        } else if(normal.y == 0.0f) {
            vertex = math::normalize(math::Vector3{-normal.z, 0.0f, normal.x});
        } else {
            vertex = math::normalize(math::Vector3{-normal.y, normal.x, 0.0f});
        }
        // Rescales to diameter of 1
        vertex *= 0.5f;
        // Rescale the circle to have d = diameter
        vertex *= diameter;
        // Generate a circle in the plane n = normal, d = 0 and center it at origin
        math::Quaternion rotated_vec = math::Quaternion(vertex.x, vertex.y, vertex.z, 0);
        anton::Array<math::Vector3> circle_points{anton::reserve, vert_count};
        for(i64 i = 0; i <= vert_count; ++i) {
            rotated_vec = rotation_quat * rotated_vec * conjugate(rotation_quat);
            circle_points.emplace_back(rotated_vec.x + origin.x, rotated_vec.y + origin.y, rotated_vec.z + origin.z);
        }
        return circle_points;
    }
} // namespace anton::gizmo
