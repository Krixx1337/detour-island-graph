#pragma once

#include "IslandGraph.h"

#include <functional>
#include <vector>

namespace detour_island_graph {

struct LinkCostContext {
    const IslandGraph& graph;
    IslandId startIsland;
    IslandId endIsland;
};

using LinkCost = std::function<float(const Link&, const LinkCostContext&)>;
using LinkFilter = std::function<bool(const Link&, const LinkCostContext&)>;
using HeuristicCost = std::function<float(const Vec3&, const Vec3&)>;

struct PathOptions {
    LinkCost linkCost;
    LinkFilter linkFilter;
    HeuristicCost heuristicCost;
};

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
    // Uses A* for the default geometric link cost. Supplying linkCost without a
    // heuristicCost switches to Dijkstra ordering so arbitrary costs stay optimal.
    [[nodiscard]] PathResult findPath(
        const IslandGraph& graph,
        IslandId startIsland,
        IslandId endIsland,
        const Vec3& startPosition,
        const Vec3& endPosition,
        PathOptions options = {}) const;
};

} // namespace detour_island_graph
