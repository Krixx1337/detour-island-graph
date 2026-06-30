#pragma once

#include "IslandGraph.h"

#include <functional>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace detour_island_graph {

struct LinkCostContext {
    const IslandGraph& graph;
    IslandId startIsland;
    IslandId endIsland;
};

struct EdgeTraversal {
    std::uint32_t edgeIndex = 0;
    Link link;
};

using LinkCost = std::function<float(const Link&, const LinkCostContext&)>;
using LinkFilter = std::function<bool(const Link&, const LinkCostContext&)>;
using HeuristicCost = std::function<float(const Vec3&, const Vec3&)>;

struct PathOptions {
    LinkCost linkCost;
    LinkFilter linkFilter;
    HeuristicCost heuristicCost;
    std::size_t maxExpandedPortals = 0;
    std::size_t maxQueuedPortals = 0;
    std::function<bool()> shouldCancel;
};

enum class PathStatus {
    Success,
    SameIsland,
    NoPath,
    InvalidIsland,
    BudgetExceeded,
    Cancelled
};

struct PathStats {
    std::size_t expandedPortals = 0;
    std::size_t queuedPortals = 0;
    std::size_t peakOpenSetSize = 0;
};

struct PathResult {
    PathStatus status = PathStatus::NoPath;
    std::vector<Link> links;
    std::vector<std::uint32_t> edgeIndices;
    float totalCost = 0.0f;
    PathStats stats;
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
