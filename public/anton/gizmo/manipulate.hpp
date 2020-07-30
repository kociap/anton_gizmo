#pragma once

#include <anton/gizmo/common.hpp>

namespace anton::gizmo {
    // translate_along_line
    //
    // Returns:
    // The translated position
    //
    [[nodiscard]] math::Vector3 translate_along_line(Ray ray, math::Vector3 axis, math::Vector3 origin);
} // namespace anton::gizmo
