#pragma once

#include "IslandGraphBuilderInternal.h"

#include <DetourNavMeshQuery.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace detour_island_graph::detail::discovery {

using SpatialCoordinate = std::int64_t;

struct Boundary {
    IslandId island = 0;
    dtPolyRef polygon = 0;
    Vec3 start;
    Vec3 end;
    Vec3 midpoint;
};

inline bool isFinite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

template <typename T>
void hashCombine(std::size_t& seed, const T& value) {
    seed ^= std::hash<T>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}

inline SpatialCoordinate quantize(float value, float cellSize) {
    const double quantized = std::floor(static_cast<double>(value) / static_cast<double>(cellSize));
    if (!std::isfinite(quantized)) {
        return 0;
    }
    if (quantized <= static_cast<double>((std::numeric_limits<SpatialCoordinate>::min)())) {
        return (std::numeric_limits<SpatialCoordinate>::min)();
    }
    if (quantized >= static_cast<double>((std::numeric_limits<SpatialCoordinate>::max)())) {
        return (std::numeric_limits<SpatialCoordinate>::max)();
    }
    return static_cast<SpatialCoordinate>(quantized);
}

inline SpatialCoordinate offsetCoordinate(SpatialCoordinate value, int offset) {
    if (offset > 0 &&
        value > (std::numeric_limits<SpatialCoordinate>::max)() - offset) {
        return (std::numeric_limits<SpatialCoordinate>::max)();
    }
    if (offset < 0 &&
        value < (std::numeric_limits<SpatialCoordinate>::min)() - offset) {
        return (std::numeric_limits<SpatialCoordinate>::min)();
    }
    return value + offset;
}

BuildStatus extractBoundaries(
    const dtNavMesh& navMesh,
    const IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    std::vector<Boundary>& output,
    std::string& message);

BuildStatus selectBoundaryRepresentatives(
    const std::vector<Boundary>& boundaries,
    const IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    std::vector<Boundary>& representatives);

bool isBetterLink(
    const Link& lhs,
    const Link& rhs,
    const IslandGraph& graph,
    const BuildConfig& config);

BuildStatus discoverCandidates(
    dtNavMeshQuery& query,
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    const std::vector<Boundary>& boundaries,
    const std::vector<Boundary>& representatives,
    BuildStats& stats,
    std::vector<Link>& candidates,
    std::string& message);

BuildStatus pruneCandidates(
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    std::vector<Link>& candidates);

} // namespace detour_island_graph::detail::discovery
