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
    const Link* link = nullptr;
    bool closed = false;
};

struct OpenEntry {
    std::size_t portal = 0;
    const Link* link = nullptr;
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

    const bool useDefaultGeometricCost = !options.linkCost;
    if (useDefaultGeometricCost) {
        options.linkCost = [](const Link& link) {
            return detail::distance(link.start, link.end);
        };
    }
    if (!options.linkFilter) {
        options.linkFilter = [](const Link&) {
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
            portalOffsets[islandIndex] + graph.islands()[islandIndex].outgoingLinks.size();
    }

    std::vector<State> states(portalOffsets.back());
    std::priority_queue<OpenEntry> open;
    const auto enqueue = [&](const Island& island, std::size_t linkIndex, float gCost, float hCost) {
        const Link& link = island.outgoingLinks[linkIndex];
        const std::size_t portalIndex = portalOffsets[island.id] + linkIndex;
        State& state = states[portalIndex];
        state.cost = gCost;
        state.link = &link;
        open.push({portalIndex, &link, gCost, gCost + hCost});
    };
    const Island& start = graph.islands()[startIsland];
    for (std::size_t linkIndex = 0; linkIndex < start.outgoingLinks.size(); ++linkIndex) {
        const Link& link = start.outgoingLinks[linkIndex];
        if (link.fromIsland != start.id || !graph.findIsland(link.toIsland) || !options.linkFilter(link)) {
            continue;
        }
        const float gapCost = options.linkCost(link);
        if (!isUsableCost(gapCost)) {
            continue;
        }
        const float gCost = detail::distance(startPosition, link.start) + gapCost;
        const float hCost = options.heuristicCost(link.end, endPosition);
        if (!isUsableCost(hCost)) {
            continue;
        }
        enqueue(start, linkIndex, gCost, hCost);
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

        const Link& currentLink = *currentEntry.link;
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
        for (std::size_t linkIndex = 0; linkIndex < nextIsland.outgoingLinks.size(); ++linkIndex) {
            const Link& nextLink = nextIsland.outgoingLinks[linkIndex];
            if (nextLink.fromIsland != nextIsland.id ||
                !graph.findIsland(nextLink.toIsland) ||
                !options.linkFilter(nextLink)) {
                continue;
            }
            const float gapCost = options.linkCost(nextLink);
            if (!isUsableCost(gapCost)) {
                continue;
            }
            const float gCost =
                currentState.cost +
                detail::distance(currentLink.end, nextLink.start) +
                gapCost;
            const std::size_t nextPortalIndex = portalOffsets[nextIsland.id] + linkIndex;
            State& nextState = states[nextPortalIndex];
            if (gCost >= nextState.cost) {
                continue;
            }
            const float hCost = options.heuristicCost(nextLink.end, endPosition);
            if (!isUsableCost(hCost)) {
                continue;
            }
            nextState.cost = gCost;
            nextState.previousPortal = currentEntry.portal;
            nextState.link = &nextLink;
            open.push({nextPortalIndex, &nextLink, gCost, gCost + hCost});
        }
    }

    if (bestGoalPortal == (std::numeric_limits<std::size_t>::max)()) {
        result.status = PathStatus::NoPath;
        return result;
    }

    for (std::size_t cursor = bestGoalPortal;
         cursor != (std::numeric_limits<std::size_t>::max)();
         cursor = states[cursor].previousPortal) {
        result.links.push_back(*states[cursor].link);
    }
    std::reverse(result.links.begin(), result.links.end());
    result.totalCost = bestGoalCost;
    result.status = PathStatus::Success;
    return result;
}

} // namespace detour_island_graph
