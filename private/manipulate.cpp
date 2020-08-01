#include <anton/gizmo/manipulate.hpp>

#include <intersection_tests.hpp>

namespace anton::gizmo {
    math::Vector3 translate_along_line(Ray const ray, math::Vector3 const axis, math::Vector3 const origin) {
        math::Vector3 const point_on_line = ray.origin - origin - axis * math::dot(ray.origin - origin, axis);
        math::Vector3 const plane_normal = math::normalize(ray.origin - point_on_line);
        f32 const plane_distance = math::dot(point_on_line, plane_normal);
        auto res = intersect_ray_plane(ray, plane_normal, plane_distance);
        if(!res) {
            return origin;
        }

        math::Vector3 const& hit_point = res.value().hit_point;
        math::Vector3 position = math::dot(hit_point - origin, axis) * axis;
        return position;
    }

    math::Vector3 translate_along_plane(Ray const ray, math::Vector3 const plane_normal, math::Vector3 const origin) {
        f32 const plane_distance = math::dot(origin, plane_normal);
        auto res = intersect_ray_plane(ray, plane_normal, plane_distance);
        if(!res) {
            return origin;
        }

        math::Vector3 const& hit_point = res.value().hit_point;
        return hit_point;
    }
} // namespace anton::gizmo
