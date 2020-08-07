#pragma once

#include <anton/gizmo/common.hpp>
#include <anton/math/quat.hpp>

namespace anton::gizmo {
    // translate_along_line
    // origin - the original position of the object
    //
    // Returns:
    // The translated position
    //
    [[nodiscard]] math::Vec3 translate_along_line(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray);

    // translate_along_plane
    // origin - the original position of the object
    //
    // Returns:
    // The translated position
    //
    [[nodiscard]] math::Vec3 translate_along_plane(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale);

    // scale_along_line
    // origin - the original position of the object
    //
    // Returns:
    // The transformed scale
    //
    [[nodiscard]] math::Vec3 scale_along_line(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale);

    // scale_along_plane
    // origin - the original position of the object
    //
    // Returns:
    // The transformed scale
    //
    [[nodiscard]] math::Vec3 scale_along_plane(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale);

    // scale_uniform_along_line
    // origin - the original position of the object
    //
    // Returns:
    // The transformed scale
    //
    [[nodiscard]] math::Vec3 scale_uniform_along_line(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale);

    // scale_uniform_along_plane
    // origin - the original position of the object
    //
    // Returns:
    // The transformed scale
    //
    [[nodiscard]] math::Vec3 scale_uniform_along_plane(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray,
                                                       math::Vec3 initial_scale);

    [[nodiscard]] math::Quat orient_turn(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Quat initial_orientation);

    [[nodiscard]] math::Quat orient_trackball(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray, math::Quat initial_orientation);
} // namespace anton::gizmo
