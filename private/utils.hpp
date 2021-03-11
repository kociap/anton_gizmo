#pragma once

#include <anton/array.hpp>
#include <anton/gizmo/common.hpp>
#include <anton/math/quat.hpp>

namespace anton::gizmo {
    // generate_circle
    // Generate a circle of diameter in the plane n = normal, d = dot(origin, normal) centered at origin.
    // The vertices are generated in clockwise order.
    //
    inline anton::Array<math::Vec3> generate_circle(math::Vec3 const origin, math::Vec3 const normal, f32 const diameter, i32 const vert_count) {
        f32 const angle = math::radians(360.0f / static_cast<f32>(vert_count));
        math::Vec3 const rotation_axis = normal * math::sin(angle / 2.0f);
        math::Quat const rotation_quat(rotation_axis.x, rotation_axis.y, rotation_axis.z, math::cos(angle / 2.0f));
        // Find a point in the plane n = normal, d = 0
        math::Vec3 vertex;
        if(normal.x == 0.0f) {
            vertex = math::normalize(math::Vec3{0.0f, -normal.z, normal.y});
        } else if(normal.y == 0.0f) {
            vertex = math::normalize(math::Vec3{-normal.z, 0.0f, normal.x});
        } else {
            vertex = math::normalize(math::Vec3{-normal.y, normal.x, 0.0f});
        }
        // Rescales to diameter of 1
        vertex *= 0.5f;
        // Rescale the circle to diameter
        vertex *= diameter;
        // Generate a circle in the plane n = normal, d = 0 and center it at origin
        math::Quat rotated_vec = math::Quat(vertex.x, vertex.y, vertex.z, 0);
        anton::Array<math::Vec3> circle_points{anton::reserve, vert_count};
        for(i64 i = 0; i <= vert_count; ++i) {
            rotated_vec = rotation_quat * rotated_vec * conjugate(rotation_quat);
            circle_points.emplace_back(rotated_vec.x + origin.x, rotated_vec.y + origin.y, rotated_vec.z + origin.z);
        }
        return circle_points;
    }

    // generate_circle
    // Generate a circle of diameter in the plane n = normal, d = dot(origin, normal) centered at origin.
    // The vertices are generated in clockwise order.
    //
    inline anton::Array<math::Vec3> generate_circle_new(math::Vec3 const& origin, math::Vec3 const& normal, f32 const radius, i32 const vert_count) {
        f32 const angle = math::two_pi / static_cast<f32>(vert_count);
        math::Quat const rotation_quat = math::Quat::from_axis_angle(normal, angle);
        // Find a point in the plane n = normal, d = 0
        math::Vec3 vertex;
        if(normal.x == 0.0f) {
            vertex = math::normalize(math::Vec3{0.0f, -normal.z, normal.y});
        } else if(normal.y == 0.0f) {
            vertex = math::normalize(math::Vec3{-normal.z, 0.0f, normal.x});
        } else {
            vertex = math::normalize(math::Vec3{-normal.y, normal.x, 0.0f});
        }
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
} // namespace anton::gizmo
