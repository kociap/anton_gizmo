#ifndef ANTON_INTERSECTION_TESTS_HPP_INCLUDE
#define ANTON_INTERSECTION_TESTS_HPP_INCLUDE

#include <anton/gizmo/common.hpp>
#include <anton/math/transform.hpp>
#include <anton/math/vector3.hpp>
#include <anton/math/vector4.hpp>
#include <anton/optional.hpp>

namespace anton::gizmo {
    class Raycast_Hit {
    public:
        math::Vector3 hit_point;
        math::Vector3 barycentric_coordinates;
        f32 distance = 0;
    };

    class OBB {
    public:
        math::Vector3 center;
        math::Vector3 local_x;
        math::Vector3 local_y;
        math::Vector3 local_z;
        math::Vector3 halfwidths;
    };

    inline Optional<Raycast_Hit> intersect_ray_plane(Ray const ray, math::Vector3 const plane_normal, f32 const plane_distance) {
        f32 const angle_cos = dot(ray.direction, plane_normal);
        f32 const coeff = (plane_distance - dot(ray.origin, plane_normal)) / angle_cos;
        if(math::abs(angle_cos) > math::epsilon && coeff >= 0.0f) {
            Raycast_Hit out;
            out.distance = coeff;
            out.hit_point = ray.origin + ray.direction * coeff;
            return out;
        } else {
            return null_optional;
        }
    }

    // direction is the vector which defines where the cone is expanding
    inline Optional<Raycast_Hit> intersect_ray_cone(Ray const ray, math::Vector3 const vertex, math::Vector3 const direction, f32 const angle_cos,
                                                    f32 const height) {
        Optional<Raycast_Hit> result = null_optional;

        math::Vector3 const ray_origin = ray.origin - vertex;
        f32 const angle_cos_square = angle_cos * angle_cos;
        math::Vector3 const ray_dir_prim = dot(ray.direction, direction) * direction;
        f32 const a = angle_cos_square * length_squared(ray.direction - ray_dir_prim) - length_squared(ray_dir_prim);
        f32 const ray_origin_prim_len = dot(ray_origin, direction);
        math::Vector3 const ray_origin_prim = ray_origin_prim_len * direction;
        f32 const b = 2.0f * angle_cos_square * dot(ray_origin - ray_origin_prim, ray.direction - ray_dir_prim) - 2.0f * dot(ray_origin_prim, ray_dir_prim);
        f32 const c = angle_cos_square * length_squared(ray_origin - ray_origin_prim) - length_squared(ray_origin_prim);
        if(a > math::epsilon || a < -math::epsilon) {
            f32 const delta = b * b - 4.0f * a * c;
            if(delta >= 0.0f) {
                f32 const delta_sqrt = math::sqrt(delta);
                f32 const t1 = 0.5f * (-b - delta_sqrt) / a;
                math::Vector3 const t1v = ray_origin + ray.direction * t1;
                if(f32 const point_height = dot(direction, t1v); t1 >= 0.0f && point_height >= 0 && point_height <= height) {
                    Raycast_Hit hit;
                    hit.distance = t1;
                    hit.hit_point = t1v + vertex;
                    result = hit;
                }

                f32 const t2 = 0.5f * (-b + delta_sqrt) / a;
                math::Vector3 const t2v = ray_origin + ray.direction * t2;
                if(f32 const point_height = dot(direction, t2v);
                   t2 >= 0.0f && point_height >= 0 && point_height <= height && (!result || t2 < result->distance)) {
                    Raycast_Hit hit;
                    hit.distance = t2;
                    hit.hit_point = t2v + vertex;
                    result = hit;
                }
            }
        } else {
            // Ray is parallel to the cone boundary
            if(b > math::epsilon || b < math::epsilon) {
                f32 const t = -c / b;
                f32 const point_height = dot(ray_origin + ray.direction * t, direction);
                if(t >= 0 && point_height <= height && point_height >= 0.0f) {
                    Raycast_Hit hit;
                    hit.distance = t;
                    hit.hit_point = ray.origin + ray.direction * t;
                    result = hit;
                }
            } else {
                // Ray is on the boundary of the infinite cone
                if(ray_origin_prim_len <= height && ray_origin_prim_len >= 0.0f) {
                    // Origin lies on the cone's coundary
                    Raycast_Hit hit;
                    hit.distance = 0.0f;
                    hit.hit_point = ray.origin;
                    result = hit;
                } else if(f32 const t = -dot(ray_origin, ray.direction); t >= 0.0f) {
                    Raycast_Hit hit;
                    hit.distance = t;
                    hit.hit_point = ray.origin + ray.direction * t;
                    result = hit;
                }
            }
        }

        if(Optional<Raycast_Hit> const hit = intersect_ray_plane({ray_origin, ray.direction}, direction, height);
           hit.holds_value() && (!result || hit->distance < result->distance) &&
           (length_squared(hit->hit_point - dot(hit->hit_point, direction) * direction) * angle_cos_square <= height * height)) {
            result = hit;
        }

        return result;
    }

    inline Optional<Raycast_Hit> intersect_ray_obb(Ray ray, OBB obb) {
        math::Matrix4 rotation =
            math::Matrix4(math::Vector4{obb.local_x, 0}, math::Vector4{obb.local_y, 0}, math::Vector4{obb.local_z, 0}, math::Vector4{0, 0, 0, 1});
        // Center OBB at 0
        math::Matrix4 obb_space = rotation * math::translate(-obb.center);
        math::Vector4 ray_dir = rotation * math::Vector4(ray.direction, 0);
        math::Vector4 ray_origin = obb_space * math::Vector4(ray.origin, 1);
        // AABB slab test
        f32 tmin = -math::infinity;
        f32 tmax = math::infinity;
        for(int i = 0; i < 3; ++i) {
            f32 tx1 = (obb.halfwidths[i] - ray_origin[i]) / ray_dir[i];
            f32 tx2 = (-obb.halfwidths[i] - ray_origin[i]) / ray_dir[i];
            tmax = math::min(tmax, math::max(tx1, tx2));
            tmin = math::max(tmin, math::min(tx1, tx2));
        }

        if(tmax >= 0 && tmax >= tmin) {
            Raycast_Hit out;
            out.distance = tmax;
            out.hit_point = ray.origin + ray.direction * tmax;
            return out;
        } else {
            return null_optional;
        }
    }

    inline Optional<Raycast_Hit> intersect_ray_cylinder(Ray const ray, math::Vector3 const vertex1, math::Vector3 const vertex2, f32 const radius) {
        Optional<Raycast_Hit> result = null_optional;

        f32 const radius_squared = radius * radius;
        math::Vector3 const ray_origin = ray.origin - vertex1;
        math::Vector3 const vert2 = vertex2 - vertex1;
        math::Vector3 const cylinder_normal = normalize(vert2);
        f32 const ray_dir_prim_len = dot(ray.direction, cylinder_normal);
        f32 const a = length_squared(ray.direction) - ray_dir_prim_len * ray_dir_prim_len;
        f32 const ray_origin_prim_len = dot(ray_origin, cylinder_normal);
        f32 const b = 2.0f * dot(ray_origin, ray.direction) - 2.0f * ray_origin_prim_len * ray_dir_prim_len;
        f32 const c = length_squared(ray_origin) - ray_origin_prim_len * ray_origin_prim_len - radius_squared;
        f32 const cap2_plane_dist = dot(vert2, -cylinder_normal);
        if(a > math::epsilon || a < -math::epsilon) {
            f32 const delta = b * b - 4 * a * c;
            if(delta >= 0.0f) {
                f32 const delta_sqrt = math::sqrt(delta);
                f32 const t1 = 0.5f * (-b - delta_sqrt) / a;
                math::Vector3 const t1v = ray_origin + ray.direction * t1;
                if(t1 >= 0.0f && dot(t1v, cylinder_normal) >= 0 && dot(t1v, -cylinder_normal) >= cap2_plane_dist) {
                    Raycast_Hit hit;
                    hit.distance = t1;
                    hit.hit_point = t1v + vertex1;
                    result = hit;
                }

                f32 const t2 = 0.5f * (-b + delta_sqrt) / a;
                math::Vector3 const t2v = ray_origin + ray.direction * t2;
                if(t2 >= 0.0f && (!result || t2 < t1) && dot(t2v, cylinder_normal) >= 0 && dot(t2v, -cylinder_normal) >= cap2_plane_dist) {
                    Raycast_Hit hit;
                    hit.distance = t2;
                    hit.hit_point = t2v + vertex1;
                    result = hit;
                }
            }
        } else {
            // Ray is parallel to the cylinder
            if(length_squared(ray_origin - ray_origin_prim_len * cylinder_normal) == radius_squared && dot(ray_origin, cylinder_normal) >= 0 &&
               dot(ray_origin, -cylinder_normal) >= cap2_plane_dist) {
                // Ray origin lies on the cylinder boundary
                Raycast_Hit hit;
                hit.distance = 0.0f;
                hit.hit_point = ray.origin;
                result = hit;
            }
        }

        // Cap 1
        {
            f32 const plane_distance = math::dot(vertex1, cylinder_normal);
            auto res = intersect_ray_plane(ray, cylinder_normal, plane_distance);
            if(res && length_squared(res->hit_point - vertex1) <= radius_squared && (!result || res->distance < result->distance)) {
                result = res;
            }
        }

        // Cap 2
        {
            f32 const plane_distance = math::dot(vertex2, cylinder_normal);
            auto res = intersect_ray_plane(ray, cylinder_normal, plane_distance);
            if(res && length_squared(res->hit_point - vertex2) <= radius_squared && (!result || res->distance < result->distance)) {
                result = res;
            }
        }

        return result;
    }

    inline Optional<Raycast_Hit> intersect_ray_cylinder_uncapped(Ray const ray, math::Vector3 const vertex1, math::Vector3 const vertex2, f32 const radius) {
        Optional<Raycast_Hit> result = null_optional;

        f32 const radius_squared = radius * radius;
        math::Vector3 const ray_origin = ray.origin - vertex1;
        math::Vector3 const vert2 = vertex2 - vertex1;
        math::Vector3 const cylinder_normal = normalize(vert2);
        f32 const ray_dir_prim_len = dot(ray.direction, cylinder_normal);
        f32 const a = length_squared(ray.direction) - ray_dir_prim_len * ray_dir_prim_len;
        f32 const ray_origin_prim_len = dot(ray_origin, cylinder_normal);
        f32 const b = 2.0f * dot(ray_origin, ray.direction) - 2.0f * ray_origin_prim_len * ray_dir_prim_len;
        f32 const c = length_squared(ray_origin) - ray_origin_prim_len * ray_origin_prim_len - radius_squared;
        f32 const cap2_plane_dist = dot(vert2, -cylinder_normal);
        if(a > math::epsilon || a < -math::epsilon) {
            f32 const delta = b * b - 4 * a * c;
            if(delta >= 0.0f) {
                f32 const delta_sqrt = math::sqrt(delta);
                f32 const t1 = 0.5f * (-b - delta_sqrt) / a;
                math::Vector3 const t1v = ray_origin + ray.direction * t1;
                if(t1 >= 0.0f && dot(t1v, cylinder_normal) >= 0 && dot(t1v, -cylinder_normal) >= cap2_plane_dist) {
                    Raycast_Hit hit;
                    hit.distance = t1;
                    hit.hit_point = t1v + vertex1;
                    result = hit;
                }

                f32 const t2 = 0.5f * (-b + delta_sqrt) / a;
                math::Vector3 const t2v = ray_origin + ray.direction * t2;
                if(t2 >= 0.0f && (!result || t2 < t1) && dot(t2v, cylinder_normal) >= 0 && dot(t2v, -cylinder_normal) >= cap2_plane_dist) {
                    Raycast_Hit hit;
                    hit.distance = t2;
                    hit.hit_point = t2v + vertex1;
                    result = hit;
                }
            }
        } else {
            // Ray is parallel to the cylinder
            if(length_squared(ray_origin - ray_origin_prim_len * cylinder_normal) == radius_squared && dot(ray_origin, cylinder_normal) >= 0 &&
               dot(ray_origin, -cylinder_normal) >= cap2_plane_dist) {
                // Ray origin lies on the cylinder boundary
                Raycast_Hit hit;
                hit.distance = 0.0f;
                hit.hit_point = ray.origin;
                result = hit;
            }
        }

        return result;
    }
} // namespace anton::gizmo

#endif // !ANTON_INTERSECTION_TESTS_HPP_INCLUDE
