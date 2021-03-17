#pragma once

#include <anton/math/mat4.hpp>
#include <anton/math/primitives.hpp>
#include <anton/math/quat.hpp>
#include <anton/math/vec3.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    // translate_along_line
    //
    // Parameters:
    // ray              - the current ray constructed by unprojecting the cursor.
    // axis             - the axis along which change from initial_ray will affect the scale. Must be nornalized.
    // origin           - the origin of the manipulation used to determine the position of the line.
    // initial_ray      - the ray at the start of the manipulation.
    // initial_position - the position at the start of the manipulation.
    // snap             - grid snapping (disabled if snap == 0). Change in position will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The translated position.
    //
    [[nodiscard]] math::Vec3 translate_along_line(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_position,
                                                  f32 snap = 0.0f);

    // translate_along_plane
    //
    // Parameters:
    // ray              - the current ray constructed by unprojecting the cursor.
    // axis             - the axis along which change from initial_ray will affect the scale. Must be nornalized.
    // origin           - the origin of the manipulation used to determine the position of the line.
    // initial_ray      - the ray at the start of the manipulation.
    // initial_position - the position at the start of the manipulation.
    // snap             - grid snapping (disabled if snap == 0). Change in position will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The translated position.
    //
    [[nodiscard]] math::Vec3 translate_along_plane(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray,
                                                   math::Vec3 initial_position, f32 snap = 0.0f);

    // scale_along_line
    // Scale in the direction of axis. The scale change is calculated based on
    // the difference between initial_ray and ray along axis.
    //
    // Parameters:
    // ray           - the current ray constructed by unprojecting the cursor.
    // axis          - the axis along which change from initial_ray will affect the scale. Must be nornalized.
    // origin        - the origin of the manipulation used to determine the position of the line.
    // initial_ray   - the ray at the start of the manipulation.
    // initial_scale - the scale at the start of the manipulation.
    // snap          - grid snapping (disabled if snap == 0). Change in scale will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The transformed scale.
    //
    [[nodiscard]] math::Vec3 scale_along_line(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale,
                                              f32 snap = 0.0f);

    // scale_along_plane
    // Scale uniformly in the directions perpendicular to plane_normal.
    // The scale change is calculated based on the difference between initial_ray
    // and ray in the plane defined by plane_normal and origin.
    //
    // Parameters:
    // ray           - the current ray constructed by unprojecting the cursor.
    // plane_normal  - normal defining the manipulation plane. Must be normalized.
    // origin        - the origin of the manipulation used to determine the position of the plane.
    // initial_ray   - the ray at the start of the manipulation.
    // initial_scale - the scale at the start of the manipulation.
    // snap          - grid snapping (disabled if snap == 0). Change in scale will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The transformed scale.
    //
    [[nodiscard]] math::Vec3 scale_along_plane(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale,
                                               f32 snap = 0.0f);

    // scale_uniform_along_line
    // Scale uniformly in all directions. The scale change is calculated based on
    // the difference between initial_ray and ray along axis.
    //
    // Parameters:
    // ray           - the current ray constructed by unprojecting the cursor.
    // axis          - the axis along which change from initial_ray will affect the scale. Must be nornalized.
    // origin        - the origin of the manipulation used to determine the position of the line.
    // initial_ray   - the ray at the start of the manipulation.
    // initial_scale - the scale at the start of the manipulation.
    // snap          - grid snapping (disabled if snap == 0). Change in scale will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The transformed scale.
    //
    [[nodiscard]] math::Vec3 scale_uniform_along_line(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale,
                                                      f32 snap = 0.0f);

    // scale_uniform_along_plane
    // Scale uniformly in all directions. The scale change is calculated based on the difference
    // between initial_ray and ray in the plane defined by plane_normal and origin.
    //
    // Parameters:
    // ray           - the current ray constructed by unprojecting the cursor.
    // plane_normal  - normal defining the manipulation plane. Must be normalized.
    // origin        - the origin of the manipulation used to determine the position of the plane.
    // initial_ray   - the ray at the start of the manipulation.
    // initial_scale - the scale at the start of the manipulation.
    // snap          - grid snapping (disabled if snap == 0). Change in scale will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The transformed scale.
    //
    [[nodiscard]] math::Vec3 scale_uniform_along_plane(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray,
                                                       math::Vec3 initial_scale, f32 snap = 0.0f);

    [[nodiscard]] math::Quat orient_turn(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Quat initial_orientation);

    [[nodiscard]] math::Quat orient_trackball(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray, math::Quat initial_orientation);
} // namespace anton::gizmo
