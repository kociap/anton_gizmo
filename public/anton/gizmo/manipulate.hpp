#pragma once

#include <anton/math/mat4.hpp>
#include <anton/math/primitives.hpp>
#include <anton/math/quat.hpp>
#include <anton/math/vec3.hpp>
#include <anton/types.hpp>

namespace anton::gizmo {
    // translate_along_line
    // Translate in the direction of axis. The position change is calculated based on
    // the difference between initial_ray and ray along axis.
    //
    // Parameters:
    // inverse_parent_transform - transform from the world space to the parent space of the object.
    //                            For manipulation of unparented objects use the identity matrix.
    //                      ray - the current ray in the world space constructed by unprojecting the cursor.
    //                     axis - the axis in the world space along which change from initial_ray will affect the position. Must be normalized.
    //                   origin - the origin of the manipulation in the world space used to determine the position of the line.
    //              initial_ray - the ray in the world space at the start of the manipulation in the world space.
    //         initial_position - the position at the start of the manipulation in the parent space of the object.
    //                     snap - grid snapping (disabled if snap == 0). Change in position will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The changed position in the parent space.
    //
    [[nodiscard]] math::Vec3 translate_along_line(math::Mat4 inverse_parent_transform, math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray,
                                                  math::Vec3 initial_position, f32 snap = 0.0f);

    // translate_along_plane
    // Translate in the plane spanned by first_axis and second_axis.
    // The position change is calculated based on the difference between initial_ray
    // and ray in the plane defined by first_axis, second_axis and origin.
    //
    // Parameters:
    // inverse_parent_transform - transform from the world space to the parent space of the object.
    //                            For manipulation of unparented objects use the identity matrix.
    //                      ray - the current ray in the world space constructed by unprojecting the cursor.
    //               first_axis - first axis defining the plane in the world space along which change from initial_ray will affect the position.
    //                            Must be nornalized. Must not be collinear with second_axis.
    //              second_axis - second axis defining the plane in the world space along which change from initial_ray will affect the position.
    //                            Must be nornalized. Must not be collinear with first_axis.
    //                   origin - the origin of the manipulation in the world space used to determine the position of the plane.
    //              initial_ray - the ray in the world space at the start of the manipulation in the world space.
    //         initial_position - the position at the start of the manipulation in the parent space of the object.
    //                     snap - grid snapping (disabled if snap == 0). Change in position will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The changed position in the parent space.
    //
    [[nodiscard]] math::Vec3 translate_along_plane(math::Mat4 inverse_parent_transform, math::Ray ray, math::Vec3 first_axis, math::Vec3 second_axis,
                                                   math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_position, f32 snap = 0.0f);

    // scale_along_line
    // Scale in the direction of axis. The scale change is calculated based on
    // the difference between initial_ray and ray along axis.
    //
    // Parameters:
    // inverse_parent_transform - transform from the world space to the parent space of the object.
    //                            For manipulation of unparented objects use the identity matrix.
    //                      ray - the current ray in the world space constructed by unprojecting the cursor.
    //               axis_world - the axis in the world space along which change from initial_ray will affect the scale. Must be normalized.
    //               axis_local - the axis in the local space along which change from initial_ray will affect the scale. Must be normalized.
    //                   origin - the origin of the manipulation in the world space used to determine the position of the line.
    //              initial_ray - the ray in the world space at the start of the manipulation in the world space.
    //         initial_position - the position at the start of the manipulation in the parent space of the object.
    //                     snap - grid snapping (disabled if snap == 0). Change in position will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The changed scale in the local space.
    //
    [[nodiscard]] math::Vec3 scale_along_line(math::Mat4 inverse_parent_transform, math::Ray ray, math::Vec3 axis_world, math::Vec3 axis_local,
                                              math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale, f32 snap = 0.0f);

    // scale_along_plane
    // Scale uniformly in the plane spanned by first_axis and second_axis.
    // The scale change is calculated based on the difference between initial_ray
    // and ray in the plane defined by first_axis, second_axis and origin.
    //
    // Parameters:
    // inverse_parent_transform - transform from the world space to the parent space of the object.
    //                            For manipulation of unparented objects use the identity matrix.
    //                      ray - the current ray in the world space constructed by unprojecting the cursor.
    //         first_axis_world - first axis defining the plane in the world space along which change from initial_ray will affect the scale.
    //                            Must be nornalized. Must not be collinear with second_axis_world.
    //         first_axis_local - first axis in the local space of the object. Must be normalized. Must not be collinear with second_axis_local.
    //        second_axis_world - second axis defining the plane in the world space along which change from initial_ray will affect the scale.
    //                            Must be nornalized. Must not be collinear with first_axis_world.
    //        second_axis_local - second axis in the local space of the object. Must be normalized. Must not be collinear with first_axis_local.
    //                   origin - the origin of the manipulation in the world space used to determine the position of the plane.
    //              initial_ray - the ray in the world space at the start of the manipulation in the world space.
    //         initial_position - the position at the start of the manipulation in the parent space of the object.
    //                     snap - grid snapping (disabled if snap == 0). Change in position will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The changed scale in the local space.
    //
    [[nodiscard]] math::Vec3 scale_along_plane(math::Mat4 inverse_parent_transform, math::Ray ray, math::Vec3 first_axis_world, math::Vec3 first_axis_local,
                                               math::Vec3 second_axis_world, math::Vec3 second_axis_local, math::Vec3 origin, math::Ray initial_ray,
                                               math::Vec3 initial_scale, f32 snap = 0.0f);

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
    // speed         - a factor to multiply the calculated change by.
    // snap          - grid snapping (disabled if snap == 0). Change in scale will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The transformed scale.
    //
    [[nodiscard]] math::Vec3 scale_uniform_along_line(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Vec3 initial_scale,
                                                      f32 speed, f32 snap = 0.0f);

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
    // speed         - a factor to multiply the calculated change by.
    // snap          - grid snapping (disabled if snap == 0). Change in scale will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The transformed scale.
    //
    [[nodiscard]] math::Vec3 scale_uniform_along_plane(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray,
                                                       math::Vec3 initial_scale, f32 speed, f32 snap = 0.0f);

    // orient_turn
    // Orient by rotating about axis. The orientation change is calculated based on the difference
    // between initial_ray and ray in the plane defined by axis and origin.
    //
    // Parameters:
    // ray                 - the current ray constructed by unprojecting the cursor.
    // axis                - the axis of rotation. Must be normalized.
    // origin              - the origin of the manipulation.
    // initial_ray         - the ray at the start of the manipulation.
    // initial_orientation - the scale at the start of the manipulation.
    // snap                - grid snapping (disabled if snap == 0). Change in orientation
    //                       will be rounded to the nearest multiple of snap.
    //
    // Returns:
    // The transformed orientation.
    //
    [[nodiscard]] math::Quat orient_turn(math::Ray ray, math::Vec3 axis, math::Vec3 origin, math::Ray initial_ray, math::Quat initial_orientation,
                                         f32 snap = 0.0f);

    // orient_trackball
    //
    //
    // Parameters:
    // ray           - the current ray constructed by unprojecting the cursor.
    // plane_normal  - normal defining the manipulation plane. Must be normalized.
    // origin        - the origin of the manipulation used to determine the position of the plane.
    // initial_ray   - the ray at the start of the manipulation.
    // initial_scale - the scale at the start of the manipulation.
    //
    // Returns:
    // The transformed orientation.
    //
    [[nodiscard]] math::Quat orient_trackball(math::Ray ray, math::Vec3 plane_normal, math::Vec3 origin, math::Ray initial_ray, math::Quat initial_orientation);
} // namespace anton::gizmo
