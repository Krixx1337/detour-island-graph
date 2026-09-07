#include "IslandGraphBuilderInternal.h"

#include "IslandGraphDiscoveryInternal.h"
#include "VectorMath.h"

#include <DetourNavMeshQuery.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace detour_island_graph::detail {
namespace {

bool resolvePolygon(
    const dtNavMesh& navMesh,
    dtPolyRef reference,
    const dtMeshTile*& tile,
    const dtPoly*& polygon) {
    tile = nullptr;
    polygon = nullptr;
    return dtStatusSucceed(navMesh.getTileAndPolyByRef(reference, &tile, &polygon)) &&
        tile &&
        polygon;
}

bool isEligiblePolygon(
    dtPolyRef reference,
    const dtMeshTile& tile,
    const dtPoly& polygon,
    const BuildConfig& config,
    const dtQueryFilter& filter) {
    const bool passesDetourFilter =
#ifdef DT_VIRTUAL_QUERYFILTER
        filter.passFilter(reference, &tile, &polygon);
#else
        (polygon.flags & filter.getIncludeFlags()) != 0 &&
        (polygon.flags & filter.getExcludeFlags()) == 0;
#endif

    return passesDetourFilter &&
        (config.polygonFilter
            ? config.polygonFilter(reference, tile, polygon)
            : polygon.getType() == DT_POLYTYPE_GROUND);
}

template <typename Visitor>
void visitNeighbors(
    const dtNavMesh& navMesh,
    const dtMeshTile& tile,
    const dtPoly& polygon,
    unsigned char edge,
    Visitor&& visit) {
    const unsigned short neighbor = polygon.neis[edge];
    if (neighbor == 0) {
        return;
    }
    if ((neighbor & DT_EXT_LINK) == 0) {
        visit(navMesh.getPolyRefBase(&tile) | static_cast<dtPolyRef>(neighbor - 1));
        return;
    }
    for (unsigned int linkIndex = polygon.firstLink;
         linkIndex != DT_NULL_LINK;
         linkIndex = tile.links[linkIndex].next) {
        const dtLink& link = tile.links[linkIndex];
        if (link.edge == edge && link.ref != 0) {
            visit(link.ref);
        }
    }
}

float lerp(float low, float high, float alpha) {
    return low + ((high - low) * alpha);
}

std::size_t percentile95(std::vector<std::size_t> values) {
    if (values.empty()) {
        return 0;
    }
    std::sort(values.begin(), values.end());
    return values[((values.size() - 1) * 95U) / 100U];
}

std::size_t massBucketIndex(float massScore) {
    if (massScore < (1.0f / 3.0f)) {
        return 0;
    }
    if (massScore < (2.0f / 3.0f)) {
        return 1;
    }
    return 2;
}

} // namespace

BuildStatus calculateMassScores(
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options) {
    auto& islands = IslandGraphAccess::islands(graph);
    if ((!config.massAware.enabled && !config.massAware.suppressSmallIslands) || islands.empty()) {
        return cancellationRequested(options) ? BuildStatus::Cancelled : BuildStatus::Success;
    }

    std::vector<float> rawMasses;
    rawMasses.reserve(islands.size());
    for (Island& island : islands) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        island.suppressed = false;
        const float spanX = island.boundsMax.x - island.boundsMin.x;
        const float spanZ = island.boundsMax.z - island.boundsMin.z;
        const float dominantSpan = (std::max)(spanX, spanZ);
        rawMasses.push_back(static_cast<float>(island.polygons.size()) * dominantSpan);
    }

    if (config.massAware.suppressSmallIslands && islands.size() >= 20) {
        const std::size_t suppressCount = static_cast<std::size_t>(
            std::floor(static_cast<float>(islands.size()) * config.massAware.suppressedIslandPercent));
        if (suppressCount > 0) {
            std::vector<std::size_t> rankedIslands;
            rankedIslands.reserve(islands.size());
            for (std::size_t index = 0; index < islands.size(); ++index) {
                rankedIslands.push_back(index);
            }
            std::sort(rankedIslands.begin(), rankedIslands.end(), [&](std::size_t lhs, std::size_t rhs) {
                if (rawMasses[lhs] != rawMasses[rhs]) {
                    return rawMasses[lhs] < rawMasses[rhs];
                }
                return islands[lhs].id < islands[rhs].id;
            });
            for (std::size_t rank = 0; rank < suppressCount; ++rank) {
                islands[rankedIslands[rank]].suppressed = true;
            }
        }
    }

    if (!config.massAware.enabled) {
        return BuildStatus::Success;
    }

    std::vector<float> sortedMasses;
    sortedMasses.reserve(rawMasses.size());
    for (float rawMass : rawMasses) {
        if (rawMass > 0.0f) {
            sortedMasses.push_back(rawMass);
        }
    }
    if (sortedMasses.empty()) {
        return BuildStatus::Success;
    }
    std::sort(sortedMasses.begin(), sortedMasses.end());
    const float percentileIndex =
        config.massAware.normalizationPercentile *
        static_cast<float>(sortedMasses.size() - 1);
    const std::size_t lowerIndex = static_cast<std::size_t>(std::floor(percentileIndex));
    const std::size_t upperIndex = (std::min)(lowerIndex + 1, sortedMasses.size() - 1);
    const float referenceMass = lerp(
        sortedMasses[lowerIndex],
        sortedMasses[upperIndex],
        percentileIndex - static_cast<float>(lowerIndex));
    const float minimumMass = sortedMasses.front();
    const float referenceLogRatio = std::log(referenceMass / minimumMass);

    for (std::size_t index = 0; index < islands.size(); ++index) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (rawMasses[index] <= 0.0f) {
            islands[index].massScore = 0.0f;
            continue;
        }
        islands[index].massScore = referenceLogRatio > 0.0f
            ? std::clamp(std::log(rawMasses[index] / minimumMass) / referenceLogRatio, 0.0f, 1.0f)
            : 1.0f;
    }
    return BuildStatus::Success;
}

BuildStatus floodFill(
    const dtNavMesh& navMesh,
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options) {
    auto& islands = IslandGraphAccess::islands(graph);
    auto& polygonToIsland = IslandGraphAccess::polygonToIsland(graph);
    dtQueryFilter filter;
    filter.setIncludeFlags(config.query.includeFlags);
    filter.setExcludeFlags(config.query.excludeFlags);
    std::queue<dtPolyRef> pending;
    for (int tileIndex = 0; tileIndex < navMesh.getMaxTiles(); ++tileIndex) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        const dtMeshTile* tile = navMesh.getTile(tileIndex);
        if (!tile || !tile->header) {
            continue;
        }
        for (int polygonIndex = 0; polygonIndex < tile->header->polyCount; ++polygonIndex) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            const dtPoly& polygon = tile->polys[polygonIndex];
            const dtPolyRef start = navMesh.getPolyRefBase(tile) | static_cast<dtPolyRef>(polygonIndex);
            if (!isEligiblePolygon(start, *tile, polygon, config, filter)) {
                continue;
            }
            if (polygonToIsland.find(start) != polygonToIsland.end()) {
                continue;
            }

            Island island;
            island.id = static_cast<IslandId>(islands.size());
            island.boundsMin = Vec3{
                (std::numeric_limits<float>::max)(),
                (std::numeric_limits<float>::max)(),
                (std::numeric_limits<float>::max)()};
            island.boundsMax = Vec3{
                (std::numeric_limits<float>::lowest)(),
                (std::numeric_limits<float>::lowest)(),
                (std::numeric_limits<float>::lowest)()};
            pending.push(start);

            while (!pending.empty()) {
                if (cancellationRequested(options)) {
                    return BuildStatus::Cancelled;
                }
                const dtPolyRef current = pending.front();
                pending.pop();
                if (polygonToIsland.find(current) != polygonToIsland.end()) {
                    continue;
                }
                const dtMeshTile* currentTile = nullptr;
                const dtPoly* currentPolygon = nullptr;
                if (!resolvePolygon(navMesh, current, currentTile, currentPolygon) ||
                    !isEligiblePolygon(current, *currentTile, *currentPolygon, config, filter)) {
                    continue;
                }

                polygonToIsland.emplace(current, island.id);
                island.polygons.push_back(current);

                Vec3 centroid;
                for (unsigned char vertexIndex = 0; vertexIndex < currentPolygon->vertCount; ++vertexIndex) {
                    const Vec3 vertex = fromDetour(&currentTile->verts[currentPolygon->verts[vertexIndex] * 3]);
                    centroid = add(centroid, vertex);
                    island.boundsMin.x = (std::min)(island.boundsMin.x, vertex.x);
                    island.boundsMin.y = (std::min)(island.boundsMin.y, vertex.y);
                    island.boundsMin.z = (std::min)(island.boundsMin.z, vertex.z);
                    island.boundsMax.x = (std::max)(island.boundsMax.x, vertex.x);
                    island.boundsMax.y = (std::max)(island.boundsMax.y, vertex.y);
                    island.boundsMax.z = (std::max)(island.boundsMax.z, vertex.z);
                }
                island.center = add(
                    island.center,
                    divide(centroid, static_cast<float>(currentPolygon->vertCount)));

                for (unsigned char edge = 0; edge < currentPolygon->vertCount; ++edge) {
                    visitNeighbors(navMesh, *currentTile, *currentPolygon, edge, [&](dtPolyRef neighbor) {
                        if (polygonToIsland.find(neighbor) == polygonToIsland.end()) {
                            pending.push(neighbor);
                        }
                    });
                }
            }

            if (!island.polygons.empty()) {
                island.center = divide(island.center, static_cast<float>(island.polygons.size()));
                islands.push_back(std::move(island));
            }
        }
    }
    return BuildStatus::Success;
}

BuildStatus calculateGraphHealthStats(
    const IslandGraph& graph,
    const BuildOptions& options,
    BuildStats& stats) {
    const std::size_t islandCount = graph.islands().size();
    std::vector<std::size_t> outgoingDegrees(islandCount);
    std::vector<std::size_t> incomingDegrees(islandCount);
    std::array<std::vector<std::size_t>, 3> bucketOutgoingDegrees;
    std::array<std::vector<std::size_t>, 3> bucketIncomingDegrees;
    std::vector<std::vector<IslandId>> neighbors(islandCount);
    std::vector<std::vector<IslandId>> reverseNeighbors(islandCount);
    double totalLinkLength = 0.0;
    const std::size_t totalLinks = graph.edges().size();

    for (const Island& island : graph.islands()) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        for (std::uint32_t edgeIndex : island.edgeIndices) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            if (edgeIndex >= graph.edges().size()) {
                continue;
            }
            const Edge& edge = graph.edges()[edgeIndex];
            const std::optional<Link> traversal = makeTraversalLink(edge, island.id);
            if (!traversal.has_value() || traversal->toIsland >= islandCount) {
                continue;
            }
            ++outgoingDegrees[island.id];
            ++incomingDegrees[traversal->toIsland];
            neighbors[island.id].push_back(traversal->toIsland);
            reverseNeighbors[traversal->toIsland].push_back(island.id);
        }
    }

    for (const Edge& edge : graph.edges()) {
        totalLinkLength += edge.horizontalDistance;
    }

    for (std::size_t island = 0; island < islandCount; ++island) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (outgoingDegrees[island] > 0) {
            ++stats.islandsWithOutgoingLinks;
        }
        if (incomingDegrees[island] > 0) {
            ++stats.islandsWithIncomingLinks;
        }
        const Island& graphIsland = graph.islands()[island];
        if (graphIsland.suppressed) {
            ++stats.smallIslandsSuppressed;
        }
        stats.totalIslandMass += static_cast<double>(graphIsland.massScore);
        const std::size_t bucket = massBucketIndex(graphIsland.massScore);
        MassBucketStats& bucketStats = stats.massBuckets[bucket];
        ++bucketStats.islandCount;
        bucketStats.outgoingLinkCount += outgoingDegrees[island];
        bucketStats.incomingLinkCount += incomingDegrees[island];
        bucketStats.totalMass += static_cast<double>(graphIsland.massScore);
        bucketOutgoingDegrees[bucket].push_back(outgoingDegrees[island]);
        bucketIncomingDegrees[bucket].push_back(incomingDegrees[island]);
        if (outgoingDegrees[island] == 0 && incomingDegrees[island] == 0) {
            ++stats.isolatedIslandCount;
            stats.isolatedIslandPolygonCount += graphIsland.polygons.size();
            stats.isolatedIslandMass += static_cast<double>(graphIsland.massScore);
            ++bucketStats.isolatedIslandCount;
        }
    }

    stats.maxOutgoingLinksOnIsland = outgoingDegrees.empty()
        ? 0
        : *(std::max_element)(outgoingDegrees.begin(), outgoingDegrees.end());
    stats.p95OutgoingLinksOnIsland = percentile95(outgoingDegrees);
    stats.maxIncomingLinksOnIsland = incomingDegrees.empty()
        ? 0
        : *(std::max_element)(incomingDegrees.begin(), incomingDegrees.end());
    stats.p95IncomingLinksOnIsland = percentile95(incomingDegrees);
    for (std::size_t bucket = 0; bucket < stats.massBuckets.size(); ++bucket) {
        MassBucketStats& bucketStats = stats.massBuckets[bucket];
        bucketStats.maxOutgoingLinksOnIsland = bucketOutgoingDegrees[bucket].empty()
            ? 0
            : *(std::max_element)(bucketOutgoingDegrees[bucket].begin(), bucketOutgoingDegrees[bucket].end());
        bucketStats.p95OutgoingLinksOnIsland = percentile95(bucketOutgoingDegrees[bucket]);
        bucketStats.maxIncomingLinksOnIsland = bucketIncomingDegrees[bucket].empty()
            ? 0
            : *(std::max_element)(bucketIncomingDegrees[bucket].begin(), bucketIncomingDegrees[bucket].end());
        bucketStats.p95IncomingLinksOnIsland = percentile95(bucketIncomingDegrees[bucket]);
    }
    stats.averageLinkLength = totalLinks > 0
        ? totalLinkLength / static_cast<double>(totalLinks)
        : 0.0;

    // Guardrail: these components are routeability diagnostics, so they must follow traversal
    // direction. A one-way edge should not make two islands look mutually reachable.
    std::vector<bool> visited(islandCount);
    std::vector<IslandId> finishOrder;
    finishOrder.reserve(islandCount);
    for (IslandId island = 0; island < islandCount; ++island) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (visited[island]) {
            continue;
        }
        std::vector<std::pair<IslandId, std::size_t>> stack;
        stack.push_back({island, 0});
        visited[island] = true;
        while (!stack.empty()) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            const IslandId current = stack.back().first;
            std::size_t& nextIndex = stack.back().second;
            if (nextIndex < neighbors[current].size()) {
                const IslandId neighbor = neighbors[current][nextIndex++];
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    stack.push_back({neighbor, 0});
                }
                continue;
            }
            finishOrder.push_back(current);
            stack.pop_back();
        }
    }

    std::fill(visited.begin(), visited.end(), false);
    for (auto orderIt = finishOrder.rbegin(); orderIt != finishOrder.rend(); ++orderIt) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (visited[*orderIt]) {
            continue;
        }
        ++stats.connectedComponentCount;
        std::size_t componentSize = 0;
        std::size_t componentPolygonCount = 0;
        double componentMass = 0.0;
        std::vector<IslandId> stack{*orderIt};
        visited[*orderIt] = true;
        while (!stack.empty()) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            const IslandId current = stack.back();
            stack.pop_back();
            ++componentSize;
            const Island& graphIsland = graph.islands()[current];
            componentPolygonCount += graphIsland.polygons.size();
            componentMass += static_cast<double>(graphIsland.massScore);
            for (IslandId neighbor : reverseNeighbors[current]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    stack.push_back(neighbor);
                }
            }
        }
        if (componentSize > stats.largestConnectedComponentIslandCount) {
            stats.largestConnectedComponentIslandCount = componentSize;
            stats.largestConnectedComponentPolygonCount = componentPolygonCount;
            stats.largestConnectedComponentMass = componentMass;
        }
    }

    // Guardrail: dominant-island and reachability diagnostics must follow the same directed
    // traversals as routing. Sorting local copies for distinct-neighbor counts leaves the
    // adjacency used above untouched.
    std::size_t totalPolygons = 0;
    for (const Island& island : graph.islands()) {
        totalPolygons += island.polygons.size();
    }
    IslandId mainlandId = 0;
    std::size_t mainlandPolygons = 0;
    bool hasMainland = false;
    for (const Island& island : graph.islands()) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (!hasMainland || island.polygons.size() > mainlandPolygons ||
            (island.polygons.size() == mainlandPolygons && island.id < mainlandId)) {
            hasMainland = true;
            mainlandId = island.id;
            mainlandPolygons = island.polygons.size();
        }
    }
    if (hasMainland && mainlandId < islandCount) {
        const Island& mainland = graph.islands()[mainlandId];
        LargestIslandStats& largest = stats.largestIsland;
        largest.id = mainlandId;
        largest.valid = true;
        largest.polygonCount = mainlandPolygons;
        largest.polygonShare = totalPolygons > 0
            ? static_cast<double>(mainlandPolygons) / static_cast<double>(totalPolygons)
            : 0.0;
        largest.outgoingTraversalCount = outgoingDegrees[mainlandId];
        largest.incomingTraversalCount = incomingDegrees[mainlandId];
        const auto distinctNeighborCount = [](std::vector<IslandId> targets, std::size_t islandCount) {
            std::sort(targets.begin(), targets.end());
            targets.erase(std::unique(targets.begin(), targets.end()), targets.end());
            std::size_t distinct = 0;
            for (IslandId target : targets) {
                if (target < islandCount) {
                    ++distinct;
                }
            }
            return distinct;
        };
        largest.distinctOutgoingNeighbors = distinctNeighborCount(neighbors[mainlandId], islandCount);
        largest.distinctIncomingNeighbors = distinctNeighborCount(reverseNeighbors[mainlandId], islandCount);
        for (std::uint32_t edgeIndex : mainland.edgeIndices) {
            if (edgeIndex < graph.edges().size()) {
                ++largest.incidentCorridorCount;
            }
        }
        if (mainlandId < stats.boundaries.representativeCountByIsland.size()) {
            largest.representativeCount = stats.boundaries.representativeCountByIsland[mainlandId];
        }
        largest.suppressed = mainland.suppressed;
        largest.massScore = mainland.massScore;

        std::vector<char> forwardVisited(islandCount, 0);
        std::vector<char> reverseVisited(islandCount, 0);
        std::vector<IslandId> stack{mainlandId};
        forwardVisited[mainlandId] = 1;
        while (!stack.empty()) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            const IslandId current = stack.back();
            stack.pop_back();
            for (IslandId neighbor : neighbors[current]) {
                if (neighbor < islandCount && !forwardVisited[neighbor]) {
                    forwardVisited[neighbor] = 1;
                    stack.push_back(neighbor);
                }
            }
        }
        stack.push_back(mainlandId);
        reverseVisited[mainlandId] = 1;
        while (!stack.empty()) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            const IslandId current = stack.back();
            stack.pop_back();
            for (IslandId neighbor : reverseNeighbors[current]) {
                if (neighbor < islandCount && !reverseVisited[neighbor]) {
                    reverseVisited[neighbor] = 1;
                    stack.push_back(neighbor);
                }
            }
        }

        ReachabilityStats& reach = stats.reachability;
        reach.mainlandId = mainlandId;
        reach.valid = true;
        const double totalPolygonDenominator = totalPolygons > 0
            ? static_cast<double>(totalPolygons)
            : 1.0;
        const std::size_t satellitePolygons = totalPolygons >= mainlandPolygons
            ? totalPolygons - mainlandPolygons
            : 0;
        const double satellitePolygonDenominator = satellitePolygons > 0
            ? static_cast<double>(satellitePolygons)
            : 1.0;
        for (IslandId island = 0; island < islandCount; ++island) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            if (island == mainlandId) {
                continue;
            }
            const Island& graphIsland = graph.islands()[island];
            const std::size_t polygons = graphIsland.polygons.size();
            const bool forward = forwardVisited[island] != 0;
            const bool reverse = reverseVisited[island] != 0;
            if (!graphIsland.suppressed) {
                ++reach.unsuppressedIslandCount;
                reach.unsuppressedPolygonCount += polygons;
            }
            if (forward) {
                ++reach.forwardReachableIslands;
                reach.forwardReachablePolygons += polygons;
            }
            if (reverse) {
                ++reach.reverseReachableIslands;
                reach.reverseReachablePolygons += polygons;
            }
            if (forward && reverse) {
                ++reach.mutualReachableIslands;
                reach.mutualReachablePolygons += polygons;
            }
            if (!forward && !reverse) {
                ++reach.unreachableIslands;
                reach.unreachablePolygons += polygons;
                if (graphIsland.suppressed) {
                    ++reach.unreachableSuppressedIslands;
                    reach.unreachableSuppressedPolygons += polygons;
                }
            }
        }
        reach.forwardReachablePolygonShare =
            static_cast<double>(reach.forwardReachablePolygons) / totalPolygonDenominator;
        reach.reverseReachablePolygonShare =
            static_cast<double>(reach.reverseReachablePolygons) / totalPolygonDenominator;
        reach.mutualReachablePolygonShare =
            static_cast<double>(reach.mutualReachablePolygons) / totalPolygonDenominator;
        reach.unreachablePolygonShare =
            static_cast<double>(reach.unreachablePolygons) / totalPolygonDenominator;
        reach.forwardSatellitePolygonShare =
            static_cast<double>(reach.forwardReachablePolygons) / satellitePolygonDenominator;
        reach.reverseSatellitePolygonShare =
            static_cast<double>(reach.reverseReachablePolygons) / satellitePolygonDenominator;
        reach.mutualSatellitePolygonShare =
            static_cast<double>(reach.mutualReachablePolygons) / satellitePolygonDenominator;
    }
    return BuildStatus::Success;
}

namespace {

struct CanonicalEdgeKey {
    IslandId lowIsland = 0;
    IslandId highIsland = 0;
    float pointAX = 0.0f;
    float pointAY = 0.0f;
    float pointAZ = 0.0f;
    float pointBX = 0.0f;
    float pointBY = 0.0f;
    float pointBZ = 0.0f;

    bool operator==(const CanonicalEdgeKey& other) const {
        return lowIsland == other.lowIsland &&
            highIsland == other.highIsland &&
            pointAX == other.pointAX &&
            pointAY == other.pointAY &&
            pointAZ == other.pointAZ &&
            pointBX == other.pointBX &&
            pointBY == other.pointBY &&
            pointBZ == other.pointBZ;
    }
};

struct CanonicalEdgeKeyHash {
    std::size_t operator()(const CanonicalEdgeKey& key) const {
        std::size_t hash = 0;
        discovery::hashCombine(hash, key.lowIsland);
        discovery::hashCombine(hash, key.highIsland);
        discovery::hashCombine(hash, key.pointAX);
        discovery::hashCombine(hash, key.pointAY);
        discovery::hashCombine(hash, key.pointAZ);
        discovery::hashCombine(hash, key.pointBX);
        discovery::hashCombine(hash, key.pointBY);
        discovery::hashCombine(hash, key.pointBZ);
        return hash;
    }
};

bool edgeIndexListed(const Island& island, std::uint32_t edgeIndex) {
    return std::find(
        island.edgeIndices.begin(),
        island.edgeIndices.end(),
        edgeIndex) != island.edgeIndices.end();
}

} // namespace

BuildStatus calculateDirectionValidity(
    const IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats) {
    DirectionValidityStats validity;
    const std::size_t islandCount = graph.islands().size();
    const auto recordExample = [&](const DirectionOffense& offense) {
        if (validity.examples.size() < DirectionValidityStats::kMaxExamples) {
            validity.examples.push_back(offense);
        }
    };

    std::vector<char> outboundIslands(islandCount, 1);
    if (config.outboundIslandFilter) {
        for (const Island& island : graph.islands()) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            if (island.id < islandCount) {
                outboundIslands[island.id] =
                    (!island.suppressed && config.outboundIslandFilter(island, graph)) ? 1 : 0;
            }
        }
    } else {
        for (const Island& island : graph.islands()) {
            if (island.id < islandCount) {
                outboundIslands[island.id] = island.suppressed ? 0 : 1;
            }
        }
    }

    std::unordered_map<CanonicalEdgeKey, std::uint32_t, CanonicalEdgeKeyHash> firstEdgeByGeometry;
    std::uint32_t edgeIndex = 0;
    for (const Edge& edge : graph.edges()) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        ++validity.corridorCount;
        if (edge.islandA >= islandCount || edge.islandB >= islandCount) {
            ++validity.invalidIslandRefCount;
            recordExample(DirectionOffense{
                edgeIndex, edge.islandA, edge.islandB,
                edge.horizontalDistance, edge.verticalDeltaAB,
                edge.traversableAB, edge.traversableBA,
                DirectionOffenseReason::InvalidIslandRef});
            ++edgeIndex;
            continue;
        }
        if (edge.islandA == edge.islandB) {
            ++validity.selfEdgeCount;
            recordExample(DirectionOffense{
                edgeIndex, edge.islandA, edge.islandB,
                edge.horizontalDistance, edge.verticalDeltaAB,
                edge.traversableAB, edge.traversableBA,
                DirectionOffenseReason::SelfEdge});
        }
        if (!discovery::isFinite(edge.pointA) || !discovery::isFinite(edge.pointB)) {
            ++validity.nonFiniteGeometryCount;
            recordExample(DirectionOffense{
                edgeIndex, edge.islandA, edge.islandB,
                edge.horizontalDistance, edge.verticalDeltaAB,
                edge.traversableAB, edge.traversableBA,
                DirectionOffenseReason::NonFiniteGeometry});
            ++edgeIndex;
            continue;
        }

        if (edge.traversableAB && edge.traversableBA) {
            ++validity.bidirectionalCount;
        } else if (edge.traversableAB) {
            ++validity.oneWayABOnlyCount;
        } else if (edge.traversableBA) {
            ++validity.oneWayBAOnlyCount;
        } else {
            ++validity.noDirectionCount;
            recordExample(DirectionOffense{
                edgeIndex, edge.islandA, edge.islandB,
                edge.horizontalDistance, edge.verticalDeltaAB,
                false, false,
                DirectionOffenseReason::NoDirection});
        }
        validity.directedTraversalCount +=
            (edge.traversableAB ? 1U : 0U) + (edge.traversableBA ? 1U : 0U);

        const double dx = static_cast<double>(edge.pointB.x) - edge.pointA.x;
        const double dz = static_cast<double>(edge.pointB.z) - edge.pointA.z;
        const double dy = static_cast<double>(edge.pointB.y) - edge.pointA.y;
        const float recomputedHorizontal =
            static_cast<float>(std::hypot(dx, dz));
        const float recomputedVertical = static_cast<float>(dy);
        if (std::fabs(recomputedHorizontal - edge.horizontalDistance) > 0.001f ||
            std::fabs(recomputedVertical - edge.verticalDeltaAB) > 0.001f) {
            ++validity.inconsistentStoredDistanceCount;
            recordExample(DirectionOffense{
                edgeIndex, edge.islandA, edge.islandB,
                edge.horizontalDistance, edge.verticalDeltaAB,
                edge.traversableAB, edge.traversableBA,
                DirectionOffenseReason::InconsistentStoredDistance});
        }

        const float maxGap = config.gapDiscovery.maxHorizontalGap;
        const float maxUp = config.gapDiscovery.maxVerticalGapUp;
        const float maxDown = config.gapDiscovery.maxVerticalGapDown;
        const bool allowsAB = discovery::withinTraversalLimits(edge.pointA, edge.pointB, config);
        const bool allowsBA = discovery::withinTraversalLimits(edge.pointB, edge.pointA, config);
        if (edge.traversableAB && !allowsAB) {
            ++validity.limitViolationABCount;
            validity.maxExcessHorizontal = (std::max)(
                validity.maxExcessHorizontal, recomputedHorizontal - maxGap);
            if (recomputedVertical > maxUp) {
                validity.maxExcessUp = (std::max)(
                    validity.maxExcessUp, recomputedVertical - maxUp);
            }
            if (recomputedVertical < -maxDown) {
                validity.maxExcessDown = (std::max)(
                    validity.maxExcessDown, -maxDown - recomputedVertical);
            }
            recordExample(DirectionOffense{
                edgeIndex, edge.islandA, edge.islandB,
                edge.horizontalDistance, edge.verticalDeltaAB,
                edge.traversableAB, edge.traversableBA,
                DirectionOffenseReason::LimitViolationAB});
        } else if (!edge.traversableAB) {
            if (allowsAB && outboundIslands[edge.islandA] != 0) {
                ++validity.missingAllowedABCount;
                recordExample(DirectionOffense{
                    edgeIndex, edge.islandA, edge.islandB,
                    edge.horizontalDistance, edge.verticalDeltaAB,
                    edge.traversableAB, edge.traversableBA,
                    DirectionOffenseReason::MissingAllowedAB});
            } else if (allowsAB) {
                ++validity.policyBlockedABCount;
            } else {
                ++validity.geometryBlockedABCount;
            }
        }
        if (edge.traversableBA && !allowsBA) {
            ++validity.limitViolationBACount;
            validity.maxExcessHorizontal = (std::max)(
                validity.maxExcessHorizontal, recomputedHorizontal - maxGap);
            if (-recomputedVertical > maxUp) {
                validity.maxExcessUp = (std::max)(
                    validity.maxExcessUp, -recomputedVertical - maxUp);
            }
            if (-recomputedVertical < -maxDown) {
                validity.maxExcessDown = (std::max)(
                    validity.maxExcessDown, -maxDown + recomputedVertical);
            }
            recordExample(DirectionOffense{
                edgeIndex, edge.islandA, edge.islandB,
                edge.horizontalDistance, edge.verticalDeltaAB,
                edge.traversableAB, edge.traversableBA,
                DirectionOffenseReason::LimitViolationBA});
        } else if (!edge.traversableBA) {
            if (allowsBA && outboundIslands[edge.islandB] != 0) {
                ++validity.missingAllowedBACount;
                recordExample(DirectionOffense{
                    edgeIndex, edge.islandA, edge.islandB,
                    edge.horizontalDistance, edge.verticalDeltaAB,
                    edge.traversableAB, edge.traversableBA,
                    DirectionOffenseReason::MissingAllowedBA});
            } else if (allowsBA) {
                ++validity.policyBlockedBACount;
            } else {
                ++validity.geometryBlockedBACount;
            }
        }

        const bool canonicalOrder = edge.islandA <= edge.islandB;
        const CanonicalEdgeKey geometryKey{
            canonicalOrder ? edge.islandA : edge.islandB,
            canonicalOrder ? edge.islandB : edge.islandA,
            canonicalOrder ? edge.pointA.x : edge.pointB.x,
            canonicalOrder ? edge.pointA.y : edge.pointB.y,
            canonicalOrder ? edge.pointA.z : edge.pointB.z,
            canonicalOrder ? edge.pointB.x : edge.pointA.x,
            canonicalOrder ? edge.pointB.y : edge.pointA.y,
            canonicalOrder ? edge.pointB.z : edge.pointA.z};
        if (!firstEdgeByGeometry.emplace(geometryKey, edgeIndex).second) {
            ++validity.exactDuplicateGeometryCount;
            recordExample(DirectionOffense{
                edgeIndex, edge.islandA, edge.islandB,
                edge.horizontalDistance, edge.verticalDeltaAB,
                edge.traversableAB, edge.traversableBA,
                DirectionOffenseReason::ExactDuplicateGeometry});
        }
        ++edgeIndex;
    }

    for (const Island& island : graph.islands()) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (island.id >= islandCount) {
            continue;
        }
        std::vector<std::uint32_t> orderedAdjacency(
            island.edgeIndices.begin(), island.edgeIndices.end());
        std::sort(orderedAdjacency.begin(), orderedAdjacency.end());
        for (std::size_t index = 1; index < orderedAdjacency.size(); ++index) {
            if (orderedAdjacency[index] == orderedAdjacency[index - 1]) {
                ++validity.duplicateAdjacencyCount;
                recordExample(DirectionOffense{
                    orderedAdjacency[index], island.id, island.id,
                    0.0f, 0.0f, false, false,
                    DirectionOffenseReason::AdjacencyMismatch});
            }
        }
        for (std::uint32_t listedIndex : island.edgeIndices) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            if (listedIndex >= graph.edges().size() ||
                !connectsIsland(graph.edges()[listedIndex], island.id)) {
                ++validity.adjacencyMismatchCount;
                recordExample(DirectionOffense{
                    listedIndex, island.id, island.id,
                    0.0f, 0.0f, false, false,
                    DirectionOffenseReason::AdjacencyMismatch});
            }
        }
    }

    edgeIndex = 0;
    for (const Edge& edge : graph.edges()) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (edge.islandA < islandCount && edge.islandB < islandCount &&
            edge.islandA != edge.islandB) {
            const Island& islandA = graph.islands()[edge.islandA];
            const Island& islandB = graph.islands()[edge.islandB];
            if (!edgeIndexListed(islandA, edgeIndex) ||
                !edgeIndexListed(islandB, edgeIndex)) {
                ++validity.adjacencyMismatchCount;
                recordExample(DirectionOffense{
                    edgeIndex, edge.islandA, edge.islandB,
                    edge.horizontalDistance, edge.verticalDeltaAB,
                    edge.traversableAB, edge.traversableBA,
                    DirectionOffenseReason::AdjacencyMismatch});
            }
        }
        ++edgeIndex;
    }

    stats.directionValidity = std::move(validity);
    return BuildStatus::Success;
}

} // namespace detour_island_graph::detail
