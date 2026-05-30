#pragma once

#include <detour_island_graph/IslandGraph.h>

#include <cmath>

namespace detour_island_graph::detail {

inline Vec3 add(const Vec3& lhs, const Vec3& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

inline Vec3 divide(const Vec3& value, float divisor) {
    return {value.x / divisor, value.y / divisor, value.z / divisor};
}

inline float horizontalDistance(const Vec3& lhs, const Vec3& rhs) {
    const float dx = lhs.x - rhs.x;
    const float dz = lhs.z - rhs.z;
    return std::sqrt((dx * dx) + (dz * dz));
}

inline float distance(const Vec3& lhs, const Vec3& rhs) {
    const float dx = lhs.x - rhs.x;
    const float dy = lhs.y - rhs.y;
    const float dz = lhs.z - rhs.z;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

inline Vec3 fromDetour(const float* value) {
    return {value[0], value[1], value[2]};
}

inline void toDetour(const Vec3& value, float* output) {
    output[0] = value.x;
    output[1] = value.y;
    output[2] = value.z;
}

} // namespace detour_island_graph::detail
