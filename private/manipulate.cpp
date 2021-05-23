#include <anton/gizmo/manipulate.hpp>

#include <intersection_tests.hpp>
#include <utils.hpp>

namespace anton::gizmo {
    math::Vec3 translate_along_line(math::Mat4 const inverse_parent_transform, math::Ray const ray, math::Vec3 const axis, math::Vec3 const origin,
                                    math::Ray const initial_ray, math::Vec3 const initial_position, f32 const snap) {
        math::Vec3 const point_on_axis = origin + axis * math::dot(ray.origin - origin, axis);
        math::Vec3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        auto const initial_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!initial_res) {
            return initial_position;
        }

        auto const res = intersect_ray_plane(ray, plane_normal, plane_distance);
        if(res) {
            math::Vec3 const delta = res->hit_point - initial_res->hit_point;
            f32 delta_length = math::dot(delta, axis);
            if(snap != 0.0f) {
                delta_length = math::round_to_nearest(delta_length, snap);
            }
            math::Vec3 const delta_snap = delta_length * axis;
            math::Vec3 const local_delta{inverse_parent_transform * math::Vec4{delta_snap, 0.0f}};
            return initial_position + local_delta;
        } else {
            return initial_position;
        }
    }

    math::Vec3 translate_along_plane(math::Mat4 const inverse_parent_transform, math::Ray const ray, math::Vec3 const first_axis, math::Vec3 const second_axis,
                                     math::Vec3 const origin, math::Ray const initial_ray, math::Vec3 const initial_position, f32 const snap) {
        // The axes are NOT necessarily perpendicular
        math::Vec3 const plane_normal = math::normalize(math::cross(first_axis, second_axis));
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        auto const initial_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!initial_res) {
            return initial_position;
        }

        auto res = intersect_ray_plane(ray, plane_normal, plane_distance);
        if(res) {
            math::Vec3 delta = res->hit_point - initial_res->hit_point;
            if(snap != 0.0f) {
                f32 delta_first = math::dot(delta, first_axis);
                delta_first = math::round_to_nearest(delta_first, snap);
                f32 delta_second = math::dot(delta, second_axis);
                delta_second = math::round_to_nearest(delta_second, snap);
                delta = delta_first * first_axis + delta_second * second_axis;
            }
            math::Vec3 const local_delta{inverse_parent_transform * math::Vec4{delta, 0.0f}};
            return initial_position + local_delta;
        } else {
            return initial_position;
        }
    }

    math::Vec3 scale_along_line(math::Ray const ray, math::Vec3 const axis_world, math::Vec3 const axis_local, math::Vec3 const origin,
                                math::Ray const initial_ray, math::Vec3 const initial_scale, f32 const snap) {
        math::Vec3 const point_on_axis = origin + axis_world * math::dot(ray.origin - origin, axis_world);
        math::Vec3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        auto const initial_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!initial_res) {
            return initial_scale;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            f32 const offset_line_length = math::dot(initial_res->hit_point - origin, axis_world);
            f32 const hit_line_length = math::dot(res->hit_point - origin, axis_world);
            f32 factor = hit_line_length / offset_line_length;
            if(snap != 0.0f) {
                factor = math::round_to_nearest(factor, snap);
            }

            // Find out how much scale is along axis
            math::Vec3 const scale_along_axis = math::dot(initial_scale, axis_local) * axis_local;
            // The rest of the scale should not be changed
            math::Vec3 const remaining_scale = initial_scale - scale_along_axis;
            return factor * scale_along_axis + remaining_scale;
        } else {
            return initial_scale;
        }
    }

    math::Vec3 scale_along_plane(math::Ray const ray, math::Vec3 const first_axis_world, math::Vec3 const first_axis_local, math::Vec3 const second_axis_world,
                                 math::Vec3 const second_axis_local, math::Vec3 const origin, math::Ray const initial_ray, math::Vec3 const initial_scale,
                                 f32 const snap) {
        // The axes are NOT necessarily perpendicular
        math::Vec3 const plane_normal = math::normalize(math::cross(first_axis_world, second_axis_world));
        f32 const plane_distance = math::dot(origin, plane_normal);
        auto const initial_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!initial_res) {
            return initial_scale;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            math::Vec3 const origin_offset = initial_res->hit_point - origin;
            math::Vec3 const origin_hit = res->hit_point - origin;
            f32 const origin_offset_length = math::length(origin_offset);
            f32 const origin_hit_length = math::length(origin_hit);
            f32 const sign = math::dot(origin_offset, origin_hit) >= 0 ? 1 : -1;
            f32 factor = sign * origin_hit_length / origin_offset_length;
            if(snap != 0.0f) {
                factor = math::round_to_nearest(factor, snap);
            }
            // Find out how much scale is not in the plane defined
            // by the local axes and should not be modified.
            math::Vec3 const plane_normal_local = math::normalize(math::cross(first_axis_local, second_axis_local));
            math::Vec3 const remaining_scale = math::dot(initial_scale, plane_normal_local) * plane_normal_local;
            // The rest of the scale should not be changed
            math::Vec3 const scale_in_plane = initial_scale - remaining_scale;
            return factor * scale_in_plane + remaining_scale;
        } else {
            return initial_scale;
        }
    }

    math::Vec3 scale_uniform_along_line(math::Ray const ray, math::Vec3 const axis, math::Vec3 const origin, math::Ray const initial_ray,
                                        math::Vec3 const initial_scale, f32 const snap) {
        math::Vec3 const point_on_axis = origin + axis * math::dot(ray.origin - origin, axis);
        math::Vec3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        auto const initial_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!initial_res) {
            return initial_scale;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            f32 const offset_line_length = math::dot(initial_res->hit_point - origin, axis);
            f32 const hit_line_length = math::dot(res->hit_point - origin, axis);
            f32 factor = hit_line_length / offset_line_length;
            if(snap != 0.0f) {
                factor = math::round_to_nearest(factor, snap);
            }
            return factor * initial_scale;
        } else {
            return initial_scale;
        }
    }

    math::Vec3 scale_uniform_along_plane(math::Ray const ray, math::Vec3 const first_axis, math::Vec3 const second_axis, math::Vec3 const origin,
                                         math::Ray const initial_ray, math::Vec3 const initial_scale, f32 const snap) {
        math::Vec3 const plane_normal = math::normalize(math::cross(first_axis, second_axis));
        f32 const plane_distance = math::dot(origin, plane_normal);
        auto const initial_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!initial_res) {
            return initial_scale;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            math::Vec3 const origin_offset = initial_res->hit_point - origin;
            math::Vec3 const origin_hit = res->hit_point - origin;
            f32 const origin_offset_length = math::length(origin_offset);
            f32 const origin_hit_length = math::length(origin_hit);
            f32 const sign = math::dot(origin_offset, origin_hit) >= 0 ? 1 : -1;
            f32 factor = sign * origin_hit_length / origin_offset_length;
            if(snap != 0.0f) {
                factor = math::round_to_nearest(factor, snap);
            }
            return factor * initial_scale;
        } else {
            return initial_scale;
        }
    }

    math::Quat orient_turn(math::Ray const ray, math::Vec3 const axis, math::Vec3 const origin, math::Ray const initial_ray,
                           math::Quat const initial_orientation, f32 const snap) {
        math::Vec3 const plane_normal = axis;
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset
        auto const offset_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!offset_res) {
            return initial_orientation;
        }

        math::Vec3 const offset = offset_res->hit_point;
        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            math::Vec3 const start = math::normalize(offset - origin);
            math::Vec3 const target = math::normalize(res->hit_point - origin);
            math::Quat const orientation_delta = math::orient_towards(start, target);
            if(snap == 0.0f) {
                return orientation_delta * initial_orientation;
            } else {
                math::Axis_Angle const axis_angle = math::to_axis_angle(orientation_delta);
                f32 const new_angle = math::round_to_nearest(axis_angle.angle, snap);
                math::Quat const new_orientation = math::Quat::from_axis_angle(axis_angle.axis, new_angle);
                return new_orientation * initial_orientation;
            }
        } else {
            return initial_orientation;
        }
    }

    math::Quat orient_trackball(math::Ray const ray, math::Vec3 const first_axis, math::Vec3 const second_axis, math::Vec3 const origin,
                                math::Ray const initial_ray, math::Quat const initial_orientation) {
        math::Vec3 const plane_normal = math::normalize(math::cross(first_axis, second_axis));
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset
        auto const offset_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!offset_res) {
            return initial_orientation;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            math::Vec3 const delta = res->hit_point - offset_res->hit_point;
            f32 const delta_len = math::length(delta);
            if(math::is_almost_zero(delta_len, 0.0001f)) {
                return initial_orientation;
            }

            math::Vec3 const delta_norm = delta / delta_len;
            math::Vec3 const axis = math::cross(plane_normal, delta_norm);
            math::Quat const orientation_delta = math::Quat::from_axis_angle(axis, delta_len);
            return orientation_delta * initial_orientation;
        } else {
            return initial_orientation;
        }
    }
} // namespace anton::gizmo
