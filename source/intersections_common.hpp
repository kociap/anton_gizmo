#ifndef ANTON_INTERSECTIONS_COMMON_HPP_INCLUDE
#define ANTON_INTERSECTIONS_COMMON_HPP_INCLUDE

#include <anton_gizmo/base_types.hpp>
#include <anton_math/transform.hpp>
#include <anton_math/vector3.hpp>
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
} // namespace anton

#endif // !ANTON_INTERSECTIONS_COMMON_HPP_INCLUDE
