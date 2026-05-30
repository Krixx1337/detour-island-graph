#pragma once

#include "IslandGraph.h"

#include <functional>
#include <vector>

namespace detour_island_graph {

using LinkCost = std::function<float(const Link&)>;
using LinkFilter = std::function<bool(const Link&)>;

enum class PathStatus {
    Success,
    SameIsland,
    NoPath,
    InvalidIsland
};

struct PathResult {
    PathStatus status = PathStatus::NoPath;
    std::vector<Link> links;
    float totalCost = 0.0f;
};

class IslandGraphPathfinder {
public:
    PathResult findPath(
        const IslandGraph& graph,
        IslandId startIsland,
        IslandId endIsland,
        const Vec3& startPosition,
        const Vec3& endPosition,
        LinkCost linkCost = {},
        LinkFilter linkFilter = {}) const;
};

} // namespace detour_island_graph
