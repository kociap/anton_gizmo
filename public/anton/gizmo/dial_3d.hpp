#pragma once

#include <anton/array.hpp>
#include <anton/gizmo/common.hpp>
#include <anton/optional.hpp>

namespace anton::gizmo {
    struct Dial_3D {
        f32 major_diameter;
        f32 minor_diameter;
    };

    // generate_dial_3d_geometry
    //
    [[nodiscard]] Array<math::Vector3> generate_dial_3d_geometry(Dial_3D const& dial, i32 vertex_count_major, i32 vertex_count_minor);

    // intersect_dial_3d
    // Tests the dial for intersection and returns the distance to the intersection point along the ray if the ray intersects the bounding volumes.
    //
    [[nodiscard]] Optional<f32> intersect_dial_3d(Ray ray, Dial_3D const& dial, math::Matrix4 const& world_transform);
} // namespace anton::gizmo