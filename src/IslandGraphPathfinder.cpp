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

struct Portal {
    const Link* link = nullptr;
};

struct State {
    float cost = (std::numeric_limits<float>::max)();
    std::size_t previousPortal = (std::numeric_limits<std::size_t>::max)();
    bool closed = false;
};

struct OpenEntry {
    std::size_t portal = 0;
    float cost = 0.0f;

    bool operator<(const OpenEntry& other) const {
        return cost > other.cost;
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
    LinkCost linkCost,
    LinkFilter linkFilter) const {
    PathResult result;
    if (!graph.findIsland(startIsland) || !graph.findIsland(endIsland)) {
        result.status = PathStatus::InvalidIsland;
        return result;
    }
    if (startIsland == endIsland) {
        result.status = PathStatus::SameIsland;
        return result;
    }

    if (!linkCost) {
        linkCost = [](const Link& link) {
            return detail::distance(link.start, link.end);
        };
    }
    if (!linkFilter) {
        linkFilter = [](const Link&) {
            return true;
        };
    }

    std::vector<Portal> portals;
    std::vector<std::vector<std::size_t>> outgoingByIsland(graph.islands().size());
    for (const Island& island : graph.islands()) {
        for (const Link& link : island.outgoingLinks) {
            if (link.fromIsland != island.id || !graph.findIsland(link.toIsland)) {
                continue;
            }
            outgoingByIsland[island.id].push_back(portals.size());
            portals.push_back({&link});
        }
    }

    std::vector<State> states(portals.size());
    std::priority_queue<OpenEntry> open;
    for (std::size_t portalIndex : outgoingByIsland[startIsland]) {
        const Link& link = *portals[portalIndex].link;
        if (!linkFilter(link)) {
            continue;
        }
        const float gapCost = linkCost(link);
        if (!isUsableCost(gapCost)) {
            continue;
        }
        states[portalIndex].cost = detail::distance(startPosition, link.start) + gapCost;
        open.push({portalIndex, states[portalIndex].cost});
    }

    std::size_t bestGoalPortal = (std::numeric_limits<std::size_t>::max)();
    float bestGoalCost = (std::numeric_limits<float>::max)();
    while (!open.empty()) {
        const OpenEntry currentEntry = open.top();
        open.pop();
        State& currentState = states[currentEntry.portal];
        if (currentState.closed || currentEntry.cost != currentState.cost) {
            continue;
        }
        currentState.closed = true;

        const Link& currentLink = *portals[currentEntry.portal].link;
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

        for (std::size_t nextPortalIndex : outgoingByIsland[currentLink.toIsland]) {
            const Link& nextLink = *portals[nextPortalIndex].link;
            if (!linkFilter(nextLink)) {
                continue;
            }
            const float gapCost = linkCost(nextLink);
            if (!isUsableCost(gapCost)) {
                continue;
            }
            const float nextCost =
                currentState.cost +
                detail::distance(currentLink.end, nextLink.start) +
                gapCost;
            State& nextState = states[nextPortalIndex];
            if (nextCost >= nextState.cost) {
                continue;
            }
            nextState.cost = nextCost;
            nextState.previousPortal = currentEntry.portal;
            open.push({nextPortalIndex, nextCost});
        }
    }

    if (bestGoalPortal == (std::numeric_limits<std::size_t>::max)()) {
        result.status = PathStatus::NoPath;
        return result;
    }

    for (std::size_t cursor = bestGoalPortal;
         cursor != (std::numeric_limits<std::size_t>::max)();
         cursor = states[cursor].previousPortal) {
        result.links.push_back(*portals[cursor].link);
    }
    std::reverse(result.links.begin(), result.links.end());
    result.totalCost = bestGoalCost;
    result.status = PathStatus::Success;
    return result;
}

} // namespace detour_island_graph
