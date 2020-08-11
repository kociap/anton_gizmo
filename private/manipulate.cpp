#include <anton/gizmo/manipulate.hpp>

#include <intersection_tests.hpp>

namespace anton::gizmo {
    math::Vec3 translate_along_line(math::Ray const ray, math::Vec3 const axis, math::Vec3 const origin, math::Ray const initial_ray) {
        math::Vec3 const point_on_axis = origin + axis * math::dot(ray.origin - origin, axis);
        math::Vec3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vec3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return origin + math::dot(res->hit_point - offset, axis) * axis;
        } else {
            return origin;
        }
    }

    math::Vec3 translate_along_plane(math::Ray const ray, math::Vec3 const plane_normal, math::Vec3 const origin, math::Ray const initial_ray) {
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vec3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return origin + res->hit_point - offset;
        } else {
            return origin;
        }
    }

    math::Vec3 scale_along_line(math::Ray const ray, math::Vec3 const axis, math::Vec3 const origin, math::Ray const initial_ray,
                                math::Vec3 const initial_scale) {
        math::Vec3 const point_on_axis = origin + axis * math::dot(ray.origin - origin, axis);
        math::Vec3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vec3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return initial_scale + math::dot(res->hit_point - offset, axis) * axis;
        } else {
            return initial_scale;
        }
    }

    math::Vec3 scale_along_plane(math::Ray const ray, math::Vec3 const plane_normal, math::Vec3 const origin, math::Ray const initial_ray,
                                 math::Vec3 const initial_scale) {
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vec3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return initial_scale + res->hit_point - offset;
        } else {
            return initial_scale;
        }
    }

    math::Vec3 scale_uniform_along_line(math::Ray const ray, math::Vec3 const axis, math::Vec3 const origin, math::Ray const initial_ray,
                                        math::Vec3 const initial_scale) {
        math::Vec3 const point_on_axis = origin + axis * math::dot(ray.origin - origin, axis);
        math::Vec3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vec3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return initial_scale + math::dot(res->hit_point - offset, axis);
        } else {
            return initial_scale;
        }
    }

    math::Vec3 scale_uniform_along_plane(math::Ray const ray, math::Vec3 const plane_normal, math::Vec3 const origin, math::Ray const initial_ray,
                                         math::Vec3 const initial_scale) {
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vec3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return initial_scale + math::length(res->hit_point - offset);
        } else {
            return initial_scale;
        }
    }

    math::Quat orient_turn(math::Ray const ray, math::Vec3 const axis, math::Vec3 const origin, math::Ray const initial_ray,
                           math::Quat const initial_orientation) {
        math::Vec3 const plane_normal = math::normalize(axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset
        auto offset_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!offset_res) {
            return initial_orientation;
        }

        math::Vec3 const offset = offset_res->hit_point;
        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            math::Vec3 const start = math::normalize(offset - origin);
            math::Vec3 const target = math::normalize(res->hit_point - origin);
            math::Quat const orientation_delta = math::orient_towards(start, target);
            return orientation_delta * initial_orientation;
        } else {
            return initial_orientation;
        }
    }

    math::Quat orient_trackball(math::Ray const ray, math::Vec3 const plane_normal, math::Vec3 const origin, math::Ray const initial_ray,
                                math::Quat const initial_orientation) {
        f32 const plane_distance = math::dot(origin, plane_normal);
        auto offset_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
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
