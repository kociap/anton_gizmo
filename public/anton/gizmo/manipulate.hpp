#pragma once

#include <anton/gizmo/common.hpp>
#include <anton/math/quaternion.hpp>

namespace anton::gizmo {
    // translate_along_line
    // origin - the original position of the object
    //
    // Returns:
    // The translated position
    //
    [[nodiscard]] math::Vector3 translate_along_line(Ray ray, math::Vector3 axis, math::Vector3 origin, Ray initial_ray);

    // translate_along_plane
    // origin - the original position of the object
    //
    // Returns:
    // The translated position
    //
    [[nodiscard]] math::Vector3 translate_along_plane(Ray ray, math::Vector3 plane_normal, math::Vector3 origin, Ray initial_ray, math::Vector3 initial_scale);

    // scale_along_line
    // origin - the original position of the object
    //
    // Returns:
    // The transformed scale
    //
    [[nodiscard]] math::Vector3 scale_along_line(Ray ray, math::Vector3 axis, math::Vector3 origin, Ray initial_ray, math::Vector3 initial_scale);

    // scale_along_plane
    // origin - the original position of the object
    //
    // Returns:
    // The transformed scale
    //
    [[nodiscard]] math::Vector3 scale_along_plane(Ray ray, math::Vector3 plane_normal, math::Vector3 origin, Ray initial_ray, math::Vector3 initial_scale);

    // scale_uniform_along_line
    // origin - the original position of the object
    //
    // Returns:
    // The transformed scale
    //
    [[nodiscard]] math::Vector3 scale_uniform_along_line(Ray ray, math::Vector3 axis, math::Vector3 origin, Ray initial_ray, math::Vector3 initial_scale);

    // scale_uniform_along_plane
    // origin - the original position of the object
    //
    // Returns:
    // The transformed scale
    //
    [[nodiscard]] math::Vector3 scale_uniform_along_plane(Ray ray, math::Vector3 plane_normal, math::Vector3 origin, Ray initial_ray,
                                                          math::Vector3 initial_scale);

    [[nodiscard]] math::Quaternion orient_turn(Ray ray, math::Vector3 axis, math::Vector3 origin, Ray initial_ray, math::Quaternion initial_orientation);

    [[nodiscard]] math::Quaternion orient_trackball(Ray ray, math::Vector3 plane_normal, math::Vector3 origin, Ray initial_ray,
                                                    math::Quaternion initial_orientation);
} // namespace anton::gizmo
