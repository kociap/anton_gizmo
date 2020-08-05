#include <anton/gizmo/manipulate.hpp>

#include <intersection_tests.hpp>

namespace anton::gizmo {
    math::Vector3 translate_along_line(Ray const ray, math::Vector3 const axis, math::Vector3 const origin, Ray const initial_ray) {
        math::Vector3 const point_on_axis = origin + axis * math::dot(ray.origin - origin, axis);
        math::Vector3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vector3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return origin + math::dot(res->hit_point - offset, axis) * axis;
        } else {
            return origin;
        }
    }

    math::Vector3 translate_along_plane(Ray const ray, math::Vector3 const plane_normal, math::Vector3 const origin, Ray const initial_ray) {
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vector3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return origin + res->hit_point - offset;
        } else {
            return origin;
        }
    }

    math::Vector3 scale_along_line(Ray const ray, math::Vector3 const axis, math::Vector3 const origin, Ray const initial_ray,
                                   math::Vector3 const initial_scale) {
        math::Vector3 const point_on_axis = origin + axis * math::dot(ray.origin - origin, axis);
        math::Vector3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vector3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return initial_scale + math::dot(res->hit_point - offset, axis) * axis;
        } else {
            return initial_scale;
        }
    }

    math::Vector3 scale_along_plane(Ray const ray, math::Vector3 const plane_normal, math::Vector3 const origin, Ray const initial_ray,
                                    math::Vector3 const initial_scale) {
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vector3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return initial_scale + res->hit_point - offset;
        } else {
            return initial_scale;
        }
    }

    math::Vector3 scale_uniform_along_line(Ray const ray, math::Vector3 const axis, math::Vector3 const origin, Ray const initial_ray,
                                           math::Vector3 const initial_scale) {
        math::Vector3 const point_on_axis = origin + axis * math::dot(ray.origin - origin, axis);
        math::Vector3 const plane_normal = math::normalize(ray.origin - point_on_axis);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vector3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return initial_scale + math::dot(res->hit_point - offset, axis);
        } else {
            return initial_scale;
        }
    }

    math::Vector3 scale_uniform_along_plane(Ray const ray, math::Vector3 const plane_normal, math::Vector3 const origin, Ray const initial_ray,
                                            math::Vector3 const initial_scale) {
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset that we'll use to prevent the center of the object from snapping to the cursor
        math::Vector3 offset;
        if(auto res = intersect_ray_plane(initial_ray, plane_normal, plane_distance)) {
            offset = res->hit_point;
        }

        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            return initial_scale + math::length(res->hit_point - offset);
        } else {
            return initial_scale;
        }
    }

    math::Quaternion orient_turn(Ray const ray, math::Vector3 const axis, math::Vector3 const origin, Ray const initial_ray,
                                 math::Quaternion const initial_orientation) {
        math::Vector3 const plane_normal = math::normalize(ray.origin - origin);
        f32 const plane_distance = math::dot(origin, plane_normal);
        // Calculate cursor offset
        auto offset_res = intersect_ray_plane(initial_ray, plane_normal, plane_distance);
        if(!offset_res) {
            return initial_orientation;
        }

        math::Vector3 const offset = offset_res->hit_point;
        if(auto res = intersect_ray_plane(ray, plane_normal, plane_distance)) {
            // We have to normalize those since we need the cos of the angle between them
            math::Vector3 const start = math::normalize(offset - origin);
            math::Vector3 const target = math::normalize(res->hit_point - origin);
            f32 const angle_cos = math::dot(target, start);
            f32 const angle = math::acos(angle_cos);
            // Find the sign of the angle
            math::Vector3 const side = math::cross(axis, start);
            f32 const angle_sign = math::sign(math::dot(side, target));
            math::Quaternion const orientation_delta = math::Quaternion::from_axis_angle(axis, angle_sign * angle);
            return orientation_delta * initial_orientation;
        } else {
            return initial_orientation;
        }
    }

    math::Quaternion orient_trackball(Ray const ray, math::Vector3 const plane_normal, math::Vector3 const origin, Ray const initial_ray,
                                      math::Quaternion const initial_orientation) {
        return initial_orientation;
    }
} // namespace anton::gizmo
