#include "IslandGraphBuilderInternal.h"

#include "VectorMath.h"

#include <DetourNavMeshQuery.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
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
    return filter.passFilter(reference, &tile, &polygon) &&
        (config.polygonFilter
            ? config.polygonFilter(reference, tile, polygon)
            : polygon.getType() == DT_POLYTYPE_GROUND);
}

dtPolyRef getNeighbor(
    const dtNavMesh& navMesh,
    const dtMeshTile& tile,
    const dtPoly& polygon,
    unsigned char edge) {
    const unsigned short neighbor = polygon.neis[edge];
    if (neighbor == 0) {
        return 0;
    }
    if ((neighbor & DT_EXT_LINK) == 0) {
        return navMesh.getPolyRefBase(&tile) | static_cast<dtPolyRef>(neighbor - 1);
    }
    for (unsigned int linkIndex = polygon.firstLink;
         linkIndex != DT_NULL_LINK;
         linkIndex = tile.links[linkIndex].next) {
        const dtLink& link = tile.links[linkIndex];
        if (link.edge == edge && link.ref != 0) {
            return link.ref;
        }
    }
    return 0;
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

} // namespace

BuildStatus calculateMassScores(
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options) {
    auto& islands = IslandGraphAccess::islands(graph);
    if (!config.massAware.enabled || islands.empty()) {
        return cancellationRequested(options) ? BuildStatus::Cancelled : BuildStatus::Success;
    }

    std::vector<float> rawMasses;
    rawMasses.reserve(islands.size());
    for (const Island& island : islands) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        const float spanX = island.boundsMax.x - island.boundsMin.x;
        const float spanZ = island.boundsMax.z - island.boundsMin.z;
        const float dominantSpan = (std::max)(spanX, spanZ);
        rawMasses.push_back(static_cast<float>(island.polygons.size()) * dominantSpan);
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
                    const dtPolyRef neighbor = getNeighbor(navMesh, *currentTile, *currentPolygon, edge);
                    if (neighbor != 0 && polygonToIsland.find(neighbor) == polygonToIsland.end()) {
                        pending.push(neighbor);
                    }
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
    std::vector<std::vector<IslandId>> neighbors(islandCount);
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
        }
    }

    for (const Edge& edge : graph.edges()) {
        if (edge.islandA < islandCount && edge.islandB < islandCount) {
            neighbors[edge.islandA].push_back(edge.islandB);
            neighbors[edge.islandB].push_back(edge.islandA);
        }
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
        if (outgoingDegrees[island] == 0 && incomingDegrees[island] == 0) {
            ++stats.isolatedIslandCount;
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
    stats.averageLinkLength = totalLinks > 0
        ? totalLinkLength / static_cast<double>(totalLinks)
        : 0.0;

    std::vector<bool> visited(islandCount);
    std::queue<IslandId> pending;
    for (IslandId island = 0; island < islandCount; ++island) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (visited[island]) {
            continue;
        }
        ++stats.connectedComponentCount;
        std::size_t componentSize = 0;
        visited[island] = true;
        pending.push(island);
        while (!pending.empty()) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            const IslandId current = pending.front();
            pending.pop();
            ++componentSize;
            for (IslandId neighbor : neighbors[current]) {
                if (cancellationRequested(options)) {
                    return BuildStatus::Cancelled;
                }
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    pending.push(neighbor);
                }
            }
        }
        stats.largestConnectedComponentIslandCount =
            (std::max)(stats.largestConnectedComponentIslandCount, componentSize);
    }
    return BuildStatus::Success;
}

} // namespace detour_island_graph::detail
