#pragma once

#include <anton/gizmo/common.hpp>

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
    [[nodiscard]] math::Vector3 translate_along_plane(Ray ray, math::Vector3 plane_normal, math::Vector3 origin, Ray initial_ray);
} // namespace anton::gizmo
