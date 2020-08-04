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
} // namespace anton::gizmo
