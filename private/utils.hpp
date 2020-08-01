#pragma once

#include <anton/array.hpp>
#include <anton/gizmo/common.hpp>
#include <anton/math/quaternion.hpp>

namespace anton::gizmo {
    // generate_circle
    // Generate a circle of diameter 1 centered at (0, 0, 0) with normal (0, 0, -1)
    //
    inline anton::Array<math::Vector3> generate_circle(i32 const vert_count) {
        f32 angle = math::radians(360.0f / static_cast<f32>(vert_count));
        math::Vector3 rotation_axis = math::Vector3(0, 0, -1) * math::sin(angle / 2.0f);
        math::Quaternion rotation_quat(rotation_axis.x, rotation_axis.y, rotation_axis.z, math::cos(angle / 2.0f));
        math::Vector3 vertex = {0, 0.5f, 0};
        math::Quaternion rotated_vec = math::Quaternion(vertex.x, vertex.y, vertex.z, 0);
        anton::Array<math::Vector3> circle_points{anton::reserve, vert_count};
        for(i64 i = 0; i <= vert_count; ++i) {
            rotated_vec = rotation_quat * rotated_vec * conjugate(rotation_quat);
            circle_points.emplace_back(rotated_vec.x, rotated_vec.y, 0.0f);
        }
        return circle_points;
    }
} // namespace anton::gizmo
