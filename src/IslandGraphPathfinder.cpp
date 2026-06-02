#include <detour_island_graph/IslandGraphPathfinder.h>

#include "VectorMath.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

namespace detour_island_graph {
namespace {

struct State {
    float cost = (std::numeric_limits<float>::max)();
    std::size_t previousPortal = (std::numeric_limits<std::size_t>::max)();
    EdgeTraversal traversal;
    bool closed = false;
};

struct OpenEntry {
    std::size_t portal = 0;
    EdgeTraversal traversal;
    float gCost = 0.0f; // true accumulated cost (for stale entry check)
    float fCost = 0.0f; // f = g + h (used ONLY for queue sorting)

    bool operator<(const OpenEntry& other) const {
        return fCost > other.fCost; // min-heap based on fCost
    }
};

bool isUsableCost(float value) {
    return std::isfinite(value) && value >= 0.0f;
}

} // namespace

PathResult IslandGraphPathfinder::findPath(
    const IslandGraph& graph,
    IslandId startIsland,
    IslandId endIsland,
    const Vec3& startPosition,
    const Vec3& endPosition,
    PathOptions options) const {
    PathResult result;
    if (!graph.findIsland(startIsland) || !graph.findIsland(endIsland)) {
        result.status = PathStatus::InvalidIsland;
        return result;
    }
    if (startIsland == endIsland) {
        result.status = PathStatus::SameIsland;
        return result;
    }

    const LinkCostContext callbackContext{graph, startIsland, endIsland};
    const bool useDefaultGeometricCost = !options.linkCost;
    if (useDefaultGeometricCost) {
        options.linkCost = [](const Link& link, const LinkCostContext&) {
            return detail::distance(link.start, link.end);
        };
    }
    if (!options.linkFilter) {
        options.linkFilter = [](const Link&, const LinkCostContext&) {
            return true;
        };
    }
    if (!options.heuristicCost) {
        options.heuristicCost = useDefaultGeometricCost
            ? HeuristicCost([](const Vec3& current, const Vec3& target) {
                return detail::distance(current, target);
            })
            : HeuristicCost([](const Vec3&, const Vec3&) {
                return 0.0f;
            });
    }

    std::vector<std::size_t> portalOffsets(graph.islands().size() + 1, 0);
    for (std::size_t islandIndex = 0; islandIndex < graph.islands().size(); ++islandIndex) {
        portalOffsets[islandIndex + 1] =
            portalOffsets[islandIndex] + graph.islands()[islandIndex].edgeIndices.size();
    }

    std::vector<State> states(portalOffsets.back());
    std::priority_queue<OpenEntry> open;
    const auto enqueue = [&](const Island& island, std::size_t edgeSlot, const EdgeTraversal& traversal, float gCost, float hCost) {
        const std::size_t portalIndex = portalOffsets[island.id] + edgeSlot;
        State& state = states[portalIndex];
        state.cost = gCost;
        state.traversal = traversal;
        open.push({portalIndex, traversal, gCost, gCost + hCost});
    };
    const Island& start = graph.islands()[startIsland];
    for (std::size_t edgeSlot = 0; edgeSlot < start.edgeIndices.size(); ++edgeSlot) {
        const std::uint32_t edgeIndex = start.edgeIndices[edgeSlot];
        if (edgeIndex >= graph.edges().size()) {
            continue;
        }
        const std::optional<Link> traversalLink = makeTraversalLink(graph.edges()[edgeIndex], start.id);
        if (!traversalLink.has_value() ||
            !graph.findIsland(traversalLink->toIsland) ||
            !options.linkFilter(*traversalLink, callbackContext)) {
            continue;
        }
        const EdgeTraversal traversal{edgeIndex, *traversalLink};
        const float gapCost = options.linkCost(traversal.link, callbackContext);
        if (!isUsableCost(gapCost)) {
            continue;
        }
        const float gCost = detail::distance(startPosition, traversal.link.start) + gapCost;
        const float hCost = options.heuristicCost(traversal.link.end, endPosition);
        if (!isUsableCost(hCost)) {
            continue;
        }
        enqueue(start, edgeSlot, traversal, gCost, hCost);
    }

    std::size_t bestGoalPortal = (std::numeric_limits<std::size_t>::max)();
    float bestGoalCost = (std::numeric_limits<float>::max)();
    while (!open.empty()) {
        const OpenEntry currentEntry = open.top();
        open.pop();
        State& currentState = states[currentEntry.portal];
        if (currentState.closed || currentEntry.gCost != currentState.cost) {
            continue;
        }
        currentState.closed = true;

        const Link& currentLink = currentEntry.traversal.link;
        if (currentLink.toIsland == endIsland) {
            const float goalCost = currentState.cost + detail::distance(currentLink.end, endPosition);
            if (goalCost < bestGoalCost) {
                bestGoalCost = goalCost;
                bestGoalPortal = currentEntry.portal;
            }
        }
        if (currentState.cost >= bestGoalCost) {
            continue;
        }

        const Island& nextIsland = graph.islands()[currentLink.toIsland];
        for (std::size_t edgeSlot = 0; edgeSlot < nextIsland.edgeIndices.size(); ++edgeSlot) {
            const std::uint32_t edgeIndex = nextIsland.edgeIndices[edgeSlot];
            if (edgeIndex >= graph.edges().size()) {
                continue;
            }
            const std::optional<Link> nextTraversalLink = makeTraversalLink(graph.edges()[edgeIndex], nextIsland.id);
            if (!nextTraversalLink.has_value() ||
                !graph.findIsland(nextTraversalLink->toIsland) ||
                !options.linkFilter(*nextTraversalLink, callbackContext)) {
                continue;
            }
            const EdgeTraversal nextTraversal{edgeIndex, *nextTraversalLink};
            const float gapCost = options.linkCost(nextTraversal.link, callbackContext);
            if (!isUsableCost(gapCost)) {
                continue;
            }
            const float gCost =
                currentState.cost +
                detail::distance(currentLink.end, nextTraversal.link.start) +
                gapCost;
            const std::size_t nextPortalIndex = portalOffsets[nextIsland.id] + edgeSlot;
            State& nextState = states[nextPortalIndex];
            if (gCost >= nextState.cost) {
                continue;
            }
            const float hCost = options.heuristicCost(nextTraversal.link.end, endPosition);
            if (!isUsableCost(hCost)) {
                continue;
            }
            nextState.cost = gCost;
            nextState.previousPortal = currentEntry.portal;
            nextState.traversal = nextTraversal;
            open.push({nextPortalIndex, nextTraversal, gCost, gCost + hCost});
        }
    }

    if (bestGoalPortal == (std::numeric_limits<std::size_t>::max)()) {
        result.status = PathStatus::NoPath;
        return result;
    }

    for (std::size_t cursor = bestGoalPortal;
         cursor != (std::numeric_limits<std::size_t>::max)();
         cursor = states[cursor].previousPortal) {
        result.links.push_back(states[cursor].traversal.link);
        result.edgeIndices.push_back(states[cursor].traversal.edgeIndex);
    }
    std::reverse(result.links.begin(), result.links.end());
    std::reverse(result.edgeIndices.begin(), result.edgeIndices.end());
    result.totalCost = bestGoalCost;
    result.status = PathStatus::Success;
    return result;
}

} // namespace detour_island_graph
