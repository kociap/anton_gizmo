#ifndef ANTON_INTERSECTION_TESTS_HPP_INCLUDE
#define ANTON_INTERSECTION_TESTS_HPP_INCLUDE

#include <anton_gizmo/base_types.hpp>
#include <anton_math/transform.hpp>
#include <anton_math/vector3.hpp>
#include <anton_math/vector4.hpp>
#include <optional>

namespace anton {
    class Raycast_Hit {
    public:
        Vector3 hit_point;
        Vector3 barycentric_coordinates;
        float distance = 0;
    };

    class OBB {
    public:
        Vector3 center;
        Vector3 local_x;
        Vector3 local_y;
        Vector3 local_z;
        Vector3 halfwidths;
    };

    inline std::optional<Raycast_Hit> intersect_ray_plane(Ray const ray, Vector3 const plane_normal, float const plane_distance) {
        float const angle_cos = dot(ray.direction, plane_normal);
        float const coeff = (plane_distance - dot(ray.origin, plane_normal)) / angle_cos;
        if (abs(angle_cos) > constants::epsilon && coeff >= 0.0f) {
            Raycast_Hit out;
            out.distance = coeff;
            out.hit_point = ray.origin + ray.direction * coeff;
            return out;
        } else {
            return std::nullopt;
        }
    }

    inline std::optional<Raycast_Hit> intersect_ray_cone(Ray const ray, Vector3 const vertex, Vector3 const direction, float const angle_cos,
                                                         float const height) {
        std::optional<Raycast_Hit> result = std::nullopt;

        Vector3 const ray_origin = ray.origin - vertex;
        float const angle_cos_square = angle_cos * angle_cos;
        Vector3 const ray_dir_prim = dot(ray.direction, direction) * direction;
        float const a = angle_cos_square * length_squared(ray.direction - ray_dir_prim) - length_squared(ray_dir_prim);
        float const ray_origin_prim_len = dot(ray_origin, direction);
        Vector3 const ray_origin_prim = ray_origin_prim_len * direction;
        float const b = 2.0f * angle_cos_square * dot(ray_origin - ray_origin_prim, ray.direction - ray_dir_prim) - 2.0f * dot(ray_origin_prim, ray_dir_prim);
        float const c = angle_cos_square * length_squared(ray_origin - ray_origin_prim) - length_squared(ray_origin_prim);
        if (a > constants::epsilon || a < -constants::epsilon) {
            float const delta = b * b - 4.0f * a * c;
            if (delta >= 0.0f) {
                float const delta_sqrt = sqrt(delta);
                float const t1 = 0.5f * (-b - delta_sqrt) / a;
                Vector3 const t1v = ray_origin + ray.direction * t1;
                if (float const point_height = dot(direction, t1v); t1 >= 0.0f && point_height >= 0 && point_height <= height) {
                    Raycast_Hit hit;
                    hit.distance = t1;
                    hit.hit_point = t1v + vertex;
                    result = hit;
                }

                float const t2 = 0.5f * (-b + delta_sqrt) / a;
                Vector3 const t2v = ray_origin + ray.direction * t2;
                if (float const point_height = dot(direction, t2v);
                    t2 >= 0.0f && point_height >= 0 && point_height <= height && (!result || t2 < result->distance)) {
                    Raycast_Hit hit;
                    hit.distance = t2;
                    hit.hit_point = t2v + vertex;
                    result = hit;
                }
            }
        } else {
            // Ray is parallel to the cone boundary
            if (b > constants::epsilon || b < constants::epsilon) {
                float const t = -c / b;
                float const point_height = dot(ray_origin + ray.direction * t, direction);
                if (t >= 0 && point_height <= height && point_height >= 0.0f) {
                    Raycast_Hit hit;
                    hit.distance = t;
                    hit.hit_point = ray.origin + ray.direction * t;
                    result = hit;
                }
            } else {
                // Ray is on the boundary of the infinite cone
                if (ray_origin_prim_len <= height && ray_origin_prim_len >= 0.0f) {
                    // Origin lies on the cone's coundary
                    Raycast_Hit hit;
                    hit.distance = 0.0f;
                    hit.hit_point = ray.origin;
                    result = hit;
                } else if (float const t = -dot(ray_origin, ray.direction); t >= 0.0f) {
                    Raycast_Hit hit;
                    hit.distance = t;
                    hit.hit_point = ray.origin + ray.direction * t;
                    result = hit;
                }
            }
        }

        if (std::optional<Raycast_Hit> const hit = intersect_ray_plane({ray_origin, ray.direction}, direction, height);
            hit.has_value() && (!result || hit->distance < result->distance) &&
            (length_squared(hit->hit_point - dot(hit->hit_point, direction) * direction) * angle_cos_square <= height * height)) {
            result = hit;
        }

        return result;
    }

    inline std::optional<Raycast_Hit> intersect_ray_obb(Ray ray, OBB obb) {
        Matrix4 rotation = Matrix4(Vector4{obb.local_x, 0}, Vector4{obb.local_y, 0}, Vector4{obb.local_z, 0}, Vector4{0, 0, 0, 1});
        // Center OBB at 0
        Matrix4 obb_space = transform::translate(-obb.center) * rotation;
        Vector4 ray_dir = Vector4(ray.direction, 0) * rotation;
        Vector4 ray_origin = Vector4(ray.origin, 1) * obb_space;
        // AABB slab test
        float tmin = -constants::infinity;
        float tmax = constants::infinity;
        for (int i = 0; i < 3; ++i) {
            float tx1 = (obb.halfwidths[i] - ray_origin[i]) / ray_dir[i];
            float tx2 = (-obb.halfwidths[i] - ray_origin[i]) / ray_dir[i];
            tmax = min(tmax, max(tx1, tx2));
            tmin = max(tmin, min(tx1, tx2));
        }

        if (tmax >= 0 && tmax >= tmin) {
            Raycast_Hit out;
            out.distance = tmax;
            out.hit_point = ray.origin + ray.direction * tmax;
            return out;
        } else {
            return std::nullopt;
        }
    }

    inline std::optional<Raycast_Hit> intersect_ray_cylinder(Ray const ray, Vector3 const vertex1, Vector3 const vertex2, float const radius) {
        std::optional<Raycast_Hit> result = std::nullopt;

        float const radius_squared = radius * radius;
        Vector3 const ray_origin = ray.origin - vertex1;
        Vector3 const vert2 = vertex2 - vertex1;
        Vector3 const cylinder_normal = normalize(vert2);
        float const ray_dir_prim_len = dot(ray.direction, cylinder_normal);
        float const a = length_squared(ray.direction) - ray_dir_prim_len * ray_dir_prim_len;
        float const ray_origin_prim_len = dot(ray_origin, cylinder_normal);
        float const b = 2.0f * dot(ray_origin, ray.direction) - 2.0f * ray_origin_prim_len * ray_dir_prim_len;
        float const c = length_squared(ray_origin) - ray_origin_prim_len * ray_origin_prim_len - radius_squared;
        float const cap2_plane_dist = dot(vert2, -cylinder_normal);
        if (a > constants::epsilon || a < -constants::epsilon) {
            float const delta = b * b - 4 * a * c;
            if (delta >= 0.0f) {
                float const delta_sqrt = sqrt(delta);
                float const t1 = 0.5f * (-b - delta_sqrt) / a;
                Vector3 const t1v = ray_origin + ray.direction * t1;
                if (t1 >= 0.0f && dot(t1v, cylinder_normal) >= 0 && dot(t1v, -cylinder_normal) >= cap2_plane_dist) {
                    Raycast_Hit hit;
                    hit.distance = t1;
                    hit.hit_point = t1v + vertex1;
                    result = hit;
                }

                float const t2 = 0.5f * (-b + delta_sqrt) / a;
                Vector3 const t2v = ray_origin + ray.direction * t2;
                if (t2 >= 0.0f && (!result || t2 < t1) && dot(t2v, cylinder_normal) >= 0 && dot(t2v, -cylinder_normal) >= cap2_plane_dist) {
                    Raycast_Hit hit;
                    hit.distance = t2;
                    hit.hit_point = t2v + vertex1;
                    result = hit;
                }
            }
        } else {
            // Ray is parallel to the cylinder
            if (length_squared(ray_origin - ray_origin_prim_len * cylinder_normal) == radius_squared && dot(ray_origin, cylinder_normal) >= 0 &&
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
            float const angle_cos = dot(ray.direction, cylinder_normal);
            float const coeff = (-dot(ray.origin, cylinder_normal)) / angle_cos;
            Vector3 const hit_point = ray_origin + ray.direction * coeff;
            if (abs(angle_cos) > constants::epsilon && coeff >= 0.0f && (!result || coeff < result->distance) && length_squared(hit_point) <= radius_squared) {
                Raycast_Hit hit;
                hit.distance = coeff;
                hit.hit_point = hit_point + vertex1;
                result = hit;
            }
        }

        // Cap 2
        {
            float const angle_cos = dot(ray.direction, -cylinder_normal);
            float const coeff = (cap2_plane_dist - dot(ray.origin, -cylinder_normal)) / angle_cos;
            Vector3 const hit_point = ray_origin + ray.direction * coeff;
            if (abs(angle_cos) > constants::epsilon && coeff >= 0.0f && (!result || coeff < result->distance) &&
                length_squared(hit_point - vert2) <= radius_squared) {
                Raycast_Hit hit;
                hit.distance = coeff;
                hit.hit_point = hit_point + vertex1;
                result = hit;
            }
        }

        return result;
    }
} // namespace anton

#endif // !ANTON_INTERSECTION_TESTS_HPP_INCLUDE
