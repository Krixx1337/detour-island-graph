#include <detour_island_graph/IslandGraphBuilder.h>

#include "VectorMath.h"

#include <DetourAlloc.h>
#include <DetourNavMeshQuery.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace detour_island_graph {

namespace detail {

struct IslandGraphAccess {
    static std::vector<Island>& islands(IslandGraph& graph) {
        return graph.m_islands;
    }

    static std::unordered_map<dtPolyRef, IslandId>& polygonToIsland(IslandGraph& graph) {
        return graph.m_polygonToIsland;
    }
};

} // namespace detail

namespace {

using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

struct QueryDeleter {
    void operator()(dtNavMeshQuery* query) const {
        if (query) {
            dtFreeNavMeshQuery(query);
        }
    }
};

struct Boundary {
    IslandId island = 0;
    dtPolyRef polygon = 0;
    Vec3 start;
    Vec3 end;
    Vec3 midpoint;
};

struct BoundaryKey {
    IslandId island = 0;
    int midpointX = 0;
    int midpointY = 0;
    int midpointZ = 0;
    int directionX = 0;
    int directionY = 0;
    int directionZ = 0;

    bool operator==(const BoundaryKey& other) const {
        return island == other.island &&
            midpointX == other.midpointX &&
            midpointY == other.midpointY &&
            midpointZ == other.midpointZ &&
            directionX == other.directionX &&
            directionY == other.directionY &&
            directionZ == other.directionZ;
    }
};

struct LinkKey {
    IslandId fromIsland = 0;
    IslandId toIsland = 0;
    int startX = 0;
    int startY = 0;
    int startZ = 0;
    int endX = 0;
    int endY = 0;
    int endZ = 0;

    bool operator==(const LinkKey& other) const {
        return fromIsland == other.fromIsland &&
            toIsland == other.toIsland &&
            startX == other.startX &&
            startY == other.startY &&
            startZ == other.startZ &&
            endX == other.endX &&
            endY == other.endY &&
            endZ == other.endZ;
    }
};

struct IslandPair {
    IslandId from = 0;
    IslandId to = 0;

    bool operator==(const IslandPair& other) const {
        return from == other.from && to == other.to;
    }
};

template <typename T>
void hashCombine(std::size_t& seed, const T& value) {
    seed ^= std::hash<T>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}

struct BoundaryKeyHash {
    std::size_t operator()(const BoundaryKey& key) const {
        std::size_t hash = 0;
        hashCombine(hash, key.island);
        hashCombine(hash, key.midpointX);
        hashCombine(hash, key.midpointY);
        hashCombine(hash, key.midpointZ);
        hashCombine(hash, key.directionX);
        hashCombine(hash, key.directionY);
        hashCombine(hash, key.directionZ);
        return hash;
    }
};

struct LinkKeyHash {
    std::size_t operator()(const LinkKey& key) const {
        std::size_t hash = 0;
        hashCombine(hash, key.fromIsland);
        hashCombine(hash, key.toIsland);
        hashCombine(hash, key.startX);
        hashCombine(hash, key.startY);
        hashCombine(hash, key.startZ);
        hashCombine(hash, key.endX);
        hashCombine(hash, key.endY);
        hashCombine(hash, key.endZ);
        return hash;
    }
};

struct IslandPairHash {
    std::size_t operator()(const IslandPair& pair) const {
        std::size_t hash = 0;
        hashCombine(hash, pair.from);
        hashCombine(hash, pair.to);
        return hash;
    }
};

struct GlobalCellKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const GlobalCellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GlobalCellKeyHash {
    std::size_t operator()(const GlobalCellKey& key) const {
        std::size_t hash = 0;
        hashCombine(hash, key.x);
        hashCombine(hash, key.y);
        hashCombine(hash, key.z);
        return hash;
    }
};

int quantize(float value, float cellSize) {
    return static_cast<int>(std::floor(value / cellSize));
}

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

bool isBoundaryEdge(const dtMeshTile& tile, const dtPoly& polygon, unsigned char edge) {
    const unsigned short neighbor = polygon.neis[edge];
    if (neighbor == 0) {
        return true;
    }
    if ((neighbor & DT_EXT_LINK) == 0) {
        return false;
    }
    for (unsigned int linkIndex = polygon.firstLink;
         linkIndex != DT_NULL_LINK;
         linkIndex = tile.links[linkIndex].next) {
        const dtLink& link = tile.links[linkIndex];
        if (link.edge == edge && link.ref != 0) {
            return false;
        }
    }
    return true;
}

bool validate(const BuildConfig& config, std::string& message) {
    const MassAwareTuning& massAware = config.massAware;
    const DensityTuning& density = config.density;
    const bool valid =
        std::isfinite(config.gapDiscovery.maxHorizontalGap) && config.gapDiscovery.maxHorizontalGap > 0.0f &&
        std::isfinite(config.gapDiscovery.maxVerticalGapUp) && config.gapDiscovery.maxVerticalGapUp >= 0.0f &&
        std::isfinite(config.gapDiscovery.maxVerticalGapDown) && config.gapDiscovery.maxVerticalGapDown >= 0.0f &&
        (!config.boundaries.deduplicationEnabled ||
            (std::isfinite(config.boundaries.deduplicationCellSize) &&
             config.boundaries.deduplicationCellSize >= 0.0f)) &&
        config.query.maxNodes > 0 &&
        config.query.maxNearbyPolygons > 0 &&
        std::isfinite(massAware.normalizationPercentile) &&
        massAware.normalizationPercentile > 0.0f &&
        massAware.normalizationPercentile <= 1.0f &&
        std::isfinite(massAware.targetPreference) &&
        massAware.targetPreference >= 0.0f &&
        std::isfinite(massAware.lowMassPruneRadiusScale) &&
        massAware.lowMassPruneRadiusScale > 0.0f &&
        std::isfinite(massAware.highMassPruneRadiusScale) &&
        massAware.highMassPruneRadiusScale > 0.0f &&
        (!density.candidateDeduplication.enabled ||
            (std::isfinite(density.candidateDeduplication.cellSizeNear) &&
             density.candidateDeduplication.cellSizeNear >= 0.0f &&
             std::isfinite(density.candidateDeduplication.cellSizeFar) &&
             density.candidateDeduplication.cellSizeFar >= 0.0f)) &&
        (!density.localPruning.enabled ||
            (std::isfinite(density.localPruning.baseRadius) &&
             density.localPruning.baseRadius >= 0.0f &&
             (!density.localPruning.enableDistanceScaling ||
                (std::isfinite(density.localPruning.distanceScale) &&
                 density.localPruning.distanceScale >= 0.0f &&
                 std::isfinite(density.localPruning.maxRadiusScale) &&
                 density.localPruning.maxRadiusScale >= 1.0f)))) &&
        (!density.globalPruning.enabled ||
            (std::isfinite(density.globalPruning.relativeCellSize) &&
             density.globalPruning.relativeCellSize > 0.0f)) &&
        (!density.spannerPruning.enabled ||
            (std::isfinite(density.spannerPruning.pathRatio) &&
             density.spannerPruning.pathRatio >= 1.0f &&
             std::isfinite(density.spannerPruning.verticalWeight) &&
             density.spannerPruning.verticalWeight >= 0.0f));
    if (!valid) {
        message = "BuildConfig values must be finite and spatial cell sizes, horizontal gap, and query capacities must be positive.";
    }
    return valid;
}

float lerp(float low, float high, float alpha) {
    return low + ((high - low) * alpha);
}

void calculateMassScores(IslandGraph& graph, const BuildConfig& config) {
    auto& islands = detail::IslandGraphAccess::islands(graph);
    if (!config.massAware.enabled || islands.empty()) {
        return;
    }

    std::vector<float> rawMasses;
    rawMasses.reserve(islands.size());
    for (const Island& island : islands) {
        const float spanX = island.boundsMax.x - island.boundsMin.x;
        const float spanZ = island.boundsMax.z - island.boundsMin.z;
        const float dominantSpan = (std::max)({spanX, spanZ, 1.0f});
        rawMasses.push_back(static_cast<float>(island.polygons.size()) * dominantSpan);
    }

    std::vector<float> sortedMasses = rawMasses;
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
    const float referenceLog = std::log((std::max)(referenceMass, 1.0f));

    for (std::size_t index = 0; index < islands.size(); ++index) {
        const float massLog = std::log((std::max)(rawMasses[index], 1.0f));
        islands[index].massScore = massLog > 0.0f && referenceLog > 0.0f
            ? (std::min)(massLog / referenceLog, 1.0f)
            : 0.0f;
    }
}

void floodFill(const dtNavMesh& navMesh, IslandGraph& graph) {
    auto& islands = detail::IslandGraphAccess::islands(graph);
    auto& polygonToIsland = detail::IslandGraphAccess::polygonToIsland(graph);
    std::queue<dtPolyRef> pending;
    for (int tileIndex = 0; tileIndex < navMesh.getMaxTiles(); ++tileIndex) {
        const dtMeshTile* tile = navMesh.getTile(tileIndex);
        if (!tile || !tile->header) {
            continue;
        }
        for (int polygonIndex = 0; polygonIndex < tile->header->polyCount; ++polygonIndex) {
            const dtPoly& polygon = tile->polys[polygonIndex];
            if (polygon.getType() != DT_POLYTYPE_GROUND) {
                continue;
            }
            const dtPolyRef start = navMesh.getPolyRefBase(tile) | static_cast<dtPolyRef>(polygonIndex);
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
                const dtPolyRef current = pending.front();
                pending.pop();
                if (polygonToIsland.find(current) != polygonToIsland.end()) {
                    continue;
                }
                const dtMeshTile* currentTile = nullptr;
                const dtPoly* currentPolygon = nullptr;
                if (!resolvePolygon(navMesh, current, currentTile, currentPolygon) ||
                    currentPolygon->getType() != DT_POLYTYPE_GROUND) {
                    continue;
                }

                polygonToIsland.emplace(current, island.id);
                island.polygons.push_back(current);

                Vec3 centroid;
                for (unsigned char vertexIndex = 0; vertexIndex < currentPolygon->vertCount; ++vertexIndex) {
                    const Vec3 vertex = detail::fromDetour(
                        &currentTile->verts[currentPolygon->verts[vertexIndex] * 3]);
                    centroid = detail::add(centroid, vertex);
                    island.boundsMin.x = (std::min)(island.boundsMin.x, vertex.x);
                    island.boundsMin.y = (std::min)(island.boundsMin.y, vertex.y);
                    island.boundsMin.z = (std::min)(island.boundsMin.z, vertex.z);
                    island.boundsMax.x = (std::max)(island.boundsMax.x, vertex.x);
                    island.boundsMax.y = (std::max)(island.boundsMax.y, vertex.y);
                    island.boundsMax.z = (std::max)(island.boundsMax.z, vertex.z);
                }
                island.center = detail::add(
                    island.center,
                    detail::divide(centroid, static_cast<float>(currentPolygon->vertCount)));

                for (unsigned char edge = 0; edge < currentPolygon->vertCount; ++edge) {
                    const dtPolyRef neighbor = getNeighbor(navMesh, *currentTile, *currentPolygon, edge);
                    if (neighbor != 0 && polygonToIsland.find(neighbor) == polygonToIsland.end()) {
                        pending.push(neighbor);
                    }
                }
            }

            if (!island.polygons.empty()) {
                island.center = detail::divide(island.center, static_cast<float>(island.polygons.size()));
                islands.push_back(std::move(island));
            }
        }
    }
}

std::vector<Boundary> extractBoundaries(
    const dtNavMesh& navMesh,
    const IslandGraph& graph,
    const BuildConfig& config,
    BuildStats& stats) {
    std::unordered_map<BoundaryKey, Boundary, BoundaryKeyHash> boundaries;
    std::vector<Boundary> output;
    for (const Island& island : graph.islands()) {
        for (dtPolyRef reference : island.polygons) {
            const dtMeshTile* tile = nullptr;
            const dtPoly* polygon = nullptr;
            if (!resolvePolygon(navMesh, reference, tile, polygon)) {
                continue;
            }
            for (unsigned char edge = 0; edge < polygon->vertCount; ++edge) {
                if (!isBoundaryEdge(*tile, *polygon, edge)) {
                    continue;
                }
                ++stats.boundaries.rawCount;
                const Vec3 start = detail::fromDetour(&tile->verts[polygon->verts[edge] * 3]);
                const Vec3 end = detail::fromDetour(
                    &tile->verts[polygon->verts[(edge + 1) % polygon->vertCount] * 3]);
                const Vec3 midpoint = detail::divide(detail::add(start, end), 2.0f);
                const Vec3 direction{end.x - start.x, end.y - start.y, end.z - start.z};
                const Boundary boundary{island.id, reference, start, end, midpoint};
                if (config.boundaries.deduplicationEnabled) {
                    const float cell = config.boundaries.effectiveDeduplicationCellSize(
                        config.gapDiscovery.maxHorizontalGap);
                    const BoundaryKey key{
                        island.id,
                        quantize(midpoint.x, cell),
                        quantize(midpoint.y, cell),
                        quantize(midpoint.z, cell),
                        quantize(direction.x, cell),
                        quantize(direction.y, cell),
                        quantize(direction.z, cell)};
                    boundaries.emplace(key, boundary);
                } else {
                    output.push_back(boundary);
                }
            }
        }
    }
    if (config.boundaries.deduplicationEnabled) {
        output.reserve(boundaries.size());
        for (const auto& entry : boundaries) {
            output.push_back(entry.second);
        }
    }
    stats.boundaries.deduplicatedCount = output.size();
    std::sort(output.begin(), output.end(), [](const Boundary& lhs, const Boundary& rhs) {
        if (lhs.island != rhs.island) return lhs.island < rhs.island;
        if (lhs.polygon != rhs.polygon) return lhs.polygon < rhs.polygon;
        if (lhs.midpoint.x != rhs.midpoint.x) return lhs.midpoint.x < rhs.midpoint.x;
        if (lhs.midpoint.y != rhs.midpoint.y) return lhs.midpoint.y < rhs.midpoint.y;
        return lhs.midpoint.z < rhs.midpoint.z;
    });
    return output;
}

float linkDistance(const Link& link) {
    return detail::distance(link.start, link.end);
}

float rankScore(const Link& link, const IslandGraph& graph, const BuildConfig& config) {
    const float preference = config.massAware.enabled
        ? config.massAware.targetPreferenceFor(graph.islands()[link.toIsland].massScore)
        : 0.0f;
    return linkDistance(link) - preference;
}

bool isBetterLink(const Link& lhs, const Link& rhs, const IslandGraph& graph, const BuildConfig& config) {
    const float lhsRank = rankScore(lhs, graph, config);
    const float rhsRank = rankScore(rhs, graph, config);
    if (lhsRank != rhsRank) return lhsRank < rhsRank;
    const float lhsDistance = linkDistance(lhs);
    const float rhsDistance = linkDistance(rhs);
    if (lhsDistance != rhsDistance) return lhsDistance < rhsDistance;
    if (lhs.fromIsland != rhs.fromIsland) return lhs.fromIsland < rhs.fromIsland;
    if (lhs.toIsland != rhs.toIsland) return lhs.toIsland < rhs.toIsland;
    if (lhs.start.x != rhs.start.x) return lhs.start.x < rhs.start.x;
    if (lhs.start.y != rhs.start.y) return lhs.start.y < rhs.start.y;
    if (lhs.start.z != rhs.start.z) return lhs.start.z < rhs.start.z;
    if (lhs.end.x != rhs.end.x) return lhs.end.x < rhs.end.x;
    if (lhs.end.y != rhs.end.y) return lhs.end.y < rhs.end.y;
    return lhs.end.z < rhs.end.z;
}

float pruneRadius(const Link& link, const IslandGraph& graph, const BuildConfig& config) {
    float scale = 1.0f;
    if (config.massAware.enabled) {
        const float targetMass = graph.islands()[link.toIsland].massScore;
        scale *= config.massAware.pruneRadiusScaleFor(targetMass);
    }
    if (config.density.localPruning.enableDistanceScaling) {
        scale *= config.density.localPruning.pruneRadiusScaleFor(link.horizontalDistance);
    }
    return config.density.localPruning.effectiveBaseRadius(config.gapDiscovery.maxHorizontalGap) * scale;
}

bool hasAcceptableIndirectRoute(
    const Link& candidate,
    const std::vector<Island>& islands,
    float pathRatio,
    float verticalWeight) {
    const auto& outgoing = islands[candidate.fromIsland].outgoingLinks;
    const auto effort = [verticalWeight](const Link& link) {
        const float weightedVertical = link.verticalDistance * verticalWeight;
        return std::sqrt(
            (link.horizontalDistance * link.horizontalDistance) +
            (weightedVertical * weightedVertical));
    };
    const float candidateDist = effort(candidate);

    for (const Link& firstHop : outgoing) {
        if (firstHop.toIsland == candidate.toIsland) {
            continue;
        }

        const auto& secondOutgoing = islands[firstHop.toIsland].outgoingLinks;
        float bestSecondHopDist = (std::numeric_limits<float>::max)();
        for (const Link& secondHop : secondOutgoing) {
            if (secondHop.toIsland == candidate.toIsland) {
                const float dist = effort(secondHop);
                if (dist < bestSecondHopDist) {
                    bestSecondHopDist = dist;
                }
            }
        }

        if (bestSecondHopDist != (std::numeric_limits<float>::max)()) {
            const float firstHopDist = effort(firstHop);
            const float indirectDist = firstHopDist + bestSecondHopDist;
            if (indirectDist <= candidateDist * pathRatio) {
                return true;
            }
        }
    }
    return false;
}

BuildStatus discoverLinks(
    const dtNavMesh& navMesh,
    IslandGraph& graph,
    const BuildConfig& config,
    BuildStats& stats,
    std::string& message) {
    std::unique_ptr<dtNavMeshQuery, QueryDeleter> query(dtAllocNavMeshQuery());
    if (!query || dtStatusFailed(query->init(&navMesh, config.query.maxNodes))) {
        message = "Failed to initialize dtNavMeshQuery.";
        return BuildStatus::QueryInitializationFailed;
    }

    const Clock::time_point boundaryStart = Clock::now();
    const std::vector<Boundary> boundaries = extractBoundaries(navMesh, graph, config, stats);
    stats.timings.boundaryExtractionMs = elapsedMilliseconds(boundaryStart);

    const Clock::time_point discoveryStart = Clock::now();
    std::unordered_map<LinkKey, Link, LinkKeyHash> deduplicated;
    std::vector<Link> candidates;
    std::vector<dtPolyRef> nearby(static_cast<std::size_t>(config.query.maxNearbyPolygons));
    const float extents[3]{
        config.gapDiscovery.maxHorizontalGap,
        (std::max)(config.gapDiscovery.maxVerticalGapUp, config.gapDiscovery.maxVerticalGapDown),
        config.gapDiscovery.maxHorizontalGap};
    const dtQueryFilter filter;

    for (const Boundary& boundary : boundaries) {
        float center[3];
        detail::toDetour(boundary.midpoint, center);
        int nearbyCount = 0;
        ++stats.queries.count;
        if (dtStatusFailed(query->queryPolygons(
                center,
                extents,
                &filter,
                nearby.data(),
                &nearbyCount,
                config.query.maxNearbyPolygons))) {
            message = "dtNavMeshQuery::queryPolygons failed.";
            return BuildStatus::QueryFailed;
        }
        stats.queries.nearbyPolygonCount += static_cast<std::size_t>(nearbyCount);
        if (nearbyCount >= config.query.maxNearbyPolygons) {
            ++stats.queries.capacityHitCount;
        }
        for (int index = 0; index < nearbyCount; ++index) {
            const dtPolyRef candidatePolygon = nearby[static_cast<std::size_t>(index)];
            if (candidatePolygon == 0 || candidatePolygon == boundary.polygon) {
                continue;
            }
            const auto target = graph.findIslandForPolygon(candidatePolygon);
            if (!target || *target == boundary.island) {
                continue;
            }
            float projected[3];
            bool overPolygon = false;
            if (dtStatusFailed(query->closestPointOnPoly(candidatePolygon, center, projected, &overPolygon))) {
                continue;
            }
            ++stats.candidates.projectedCount;
            (void)overPolygon;
            Link link;
            link.fromIsland = boundary.island;
            link.toIsland = *target;
            link.start = boundary.midpoint;
            link.end = detail::fromDetour(projected);
            link.horizontalDistance = detail::horizontalDistance(link.start, link.end);
            link.verticalDistance = link.end.y - link.start.y;
            if (link.horizontalDistance > config.gapDiscovery.maxHorizontalGap ||
                link.verticalDistance > config.gapDiscovery.maxVerticalGapUp ||
                link.verticalDistance < -config.gapDiscovery.maxVerticalGapDown) {
                continue;
            }
            if (!config.density.candidateDeduplication.enabled) {
                candidates.push_back(link);
                continue;
            }
            const float cell = config.density.candidateDeduplication.cellSizeFor(
                    link.horizontalDistance,
                    config.gapDiscovery.maxHorizontalGap);
            const LinkKey key{
                link.fromIsland,
                link.toIsland,
                quantize(link.start.x, cell),
                quantize(link.start.y, cell),
                quantize(link.start.z, cell),
                quantize(link.end.x, cell),
                quantize(link.end.y, cell),
                quantize(link.end.z, cell)};
            const auto existing = deduplicated.find(key);
            if (existing == deduplicated.end() || isBetterLink(link, existing->second, graph, config)) {
                deduplicated[key] = link;
            }
        }
    }
    if (config.density.candidateDeduplication.enabled) {
        candidates.reserve(deduplicated.size());
        for (const auto& entry : deduplicated) {
            candidates.push_back(entry.second);
        }
    }
    stats.candidates.deduplicatedCount = candidates.size();
    stats.timings.linkDiscoveryMs = elapsedMilliseconds(discoveryStart);

    const Clock::time_point pruningStart = Clock::now();
    std::sort(candidates.begin(), candidates.end(), [&](const Link& lhs, const Link& rhs) {
        return isBetterLink(lhs, rhs, graph, config);
    });

    std::unordered_set<GlobalCellKey, GlobalCellKeyHash> occupiedGlobalCells;
    std::unordered_map<IslandPair, std::vector<Link>, IslandPairHash> acceptedByPair;
    auto& islands = detail::IslandGraphAccess::islands(graph);
    for (const Link& candidate : candidates) {
        if (config.density.globalPruning.enabled) {
            const float cellSize = config.density.globalPruning.cellSizeFor(candidate.horizontalDistance);
            const GlobalCellKey startCell{
                quantize(candidate.start.x, cellSize),
                quantize(candidate.start.y, cellSize),
                quantize(candidate.start.z, cellSize)
            };
            const GlobalCellKey endCell{
                quantize(candidate.end.x, cellSize),
                quantize(candidate.end.y, cellSize),
                quantize(candidate.end.z, cellSize)
            };
            if (occupiedGlobalCells.count(startCell) > 0 || occupiedGlobalCells.count(endCell) > 0) {
                ++stats.candidates.globalPruningRejectCount;
                continue;
            }
        }

        if (config.density.spannerPruning.enabled) {
            if (hasAcceptableIndirectRoute(
                    candidate,
                    islands,
                    config.density.spannerPruning.pathRatio,
                    config.density.spannerPruning.verticalWeight)) {
                ++stats.candidates.spannerPruningRejectCount;
                continue;
            }
        }

        bool duplicate = false;
        if (config.density.localPruning.enabled) {
            auto& accepted = acceptedByPair[{candidate.fromIsland, candidate.toIsland}];
            const float radius = pruneRadius(candidate, graph, config);
            duplicate = std::any_of(accepted.begin(), accepted.end(), [&](const Link& existing) {
                return detail::distance(candidate.start, existing.start) <= radius &&
                    detail::distance(candidate.end, existing.end) <= radius;
            });
            if (!duplicate) {
                accepted.push_back(candidate);
            } else {
                ++stats.candidates.localPruningRejectCount;
            }
        }
        if (!duplicate) {
            islands[candidate.fromIsland].outgoingLinks.push_back(candidate);
            ++stats.candidates.acceptedLinkCount;

            if (config.density.globalPruning.enabled) {
                const float cellSize = config.density.globalPruning.cellSizeFor(candidate.horizontalDistance);
                const GlobalCellKey startCell{
                    quantize(candidate.start.x, cellSize),
                    quantize(candidate.start.y, cellSize),
                    quantize(candidate.start.z, cellSize)
                };
                const GlobalCellKey endCell{
                    quantize(candidate.end.x, cellSize),
                    quantize(candidate.end.y, cellSize),
                    quantize(candidate.end.z, cellSize)
                };
                occupiedGlobalCells.insert(startCell);
                occupiedGlobalCells.insert(endCell);
            }
        }
    }
    stats.timings.pruningMs = elapsedMilliseconds(pruningStart);
    return BuildStatus::Success;
}

} // namespace

BuildResult IslandGraphBuilder::build(const dtNavMesh& navMesh, const BuildConfig& config) const {
    BuildResult result;
    const Clock::time_point totalStart = Clock::now();
    if (!validate(config, result.message)) {
        result.status = BuildStatus::InvalidConfiguration;
        result.stats.timings.totalMs = elapsedMilliseconds(totalStart);
        return result;
    }

    const Clock::time_point floodFillStart = Clock::now();
    floodFill(navMesh, result.graph);
    result.stats.timings.floodFillMs = elapsedMilliseconds(floodFillStart);
    result.stats.islandCount = result.graph.islands().size();
    for (const Island& island : result.graph.islands()) {
        result.stats.polygonCount += island.polygons.size();
    }

    const Clock::time_point massScoringStart = Clock::now();
    calculateMassScores(result.graph, config);
    result.stats.timings.massScoringMs = elapsedMilliseconds(massScoringStart);

    result.status = discoverLinks(navMesh, result.graph, config, result.stats, result.message);

    double totalLinkLength = 0.0;
    std::size_t totalLinks = 0;
    for (const Island& island : result.graph.islands()) {
        result.stats.maxOutgoingLinksOnIsland =
            (std::max)(result.stats.maxOutgoingLinksOnIsland, island.outgoingLinks.size());
        for (const Link& link : island.outgoingLinks) {
            totalLinkLength += link.horizontalDistance;
            ++totalLinks;
        }
    }
    result.stats.averageLinkLength = totalLinks > 0 ? totalLinkLength / static_cast<double>(totalLinks) : 0.0;

    result.stats.timings.totalMs = elapsedMilliseconds(totalStart);
    return result;
}

} // namespace detour_island_graph
