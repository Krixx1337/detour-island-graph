#include "IslandGraphDiscoveryInternal.h"

#include "VectorMath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace detour_island_graph::detail::discovery {
namespace {

struct BoundaryKey {
    IslandId island = 0;
    // Guardrail: keep lateral sparsity, but do not collapse every stacked 3D ledge into one
    // identity. Huge layered maps need a small amount of height awareness to preserve distinct exits.
    SpatialCoordinate midpointX = 0;
    SpatialCoordinate midpointY = 0;
    SpatialCoordinate midpointZ = 0;
    SpatialCoordinate directionX = 0;
    SpatialCoordinate directionZ = 0;

    bool operator==(const BoundaryKey& other) const {
        return island == other.island &&
            midpointX == other.midpointX &&
            midpointY == other.midpointY &&
            midpointZ == other.midpointZ &&
            directionX == other.directionX &&
            directionZ == other.directionZ;
    }
};

struct BoundaryKeyHash {
    std::size_t operator()(const BoundaryKey& key) const {
        std::size_t hash = 0;
        hashCombine(hash, key.island);
        hashCombine(hash, key.midpointX);
        hashCombine(hash, key.midpointY);
        hashCombine(hash, key.midpointZ);
        hashCombine(hash, key.directionX);
        hashCombine(hash, key.directionZ);
        return hash;
    }
};

struct BoundaryRepresentativeKey {
    IslandId island = 0;
    // Guardrail: representative reduction should still collapse noisy nearby samples, but it must
    // keep clearly separated vertical layers for maps with stacked traversal routes.
    SpatialCoordinate midpointX = 0;
    SpatialCoordinate midpointY = 0;
    SpatialCoordinate midpointZ = 0;
    int directionBucket = 0;

    bool operator==(const BoundaryRepresentativeKey& other) const {
        return island == other.island &&
            midpointX == other.midpointX &&
            midpointY == other.midpointY &&
            midpointZ == other.midpointZ &&
            directionBucket == other.directionBucket;
    }
};

struct BoundaryRepresentativeKeyHash {
    std::size_t operator()(const BoundaryRepresentativeKey& key) const {
        std::size_t hash = 0;
        hashCombine(hash, key.island);
        hashCombine(hash, key.midpointX);
        hashCombine(hash, key.midpointY);
        hashCombine(hash, key.midpointZ);
        hashCombine(hash, key.directionBucket);
        return hash;
    }
};

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

bool boundaryLess(const Boundary& lhs, const Boundary& rhs) {
    if (lhs.island != rhs.island) return lhs.island < rhs.island;
    if (lhs.polygon != rhs.polygon) return lhs.polygon < rhs.polygon;
    if (lhs.midpoint.x != rhs.midpoint.x) return lhs.midpoint.x < rhs.midpoint.x;
    if (lhs.midpoint.y != rhs.midpoint.y) return lhs.midpoint.y < rhs.midpoint.y;
    return lhs.midpoint.z < rhs.midpoint.z;
}

bool boundarySpatialLess(const Boundary& lhs, const Boundary& rhs) {
    if (lhs.midpoint.x != rhs.midpoint.x) return lhs.midpoint.x < rhs.midpoint.x;
    if (lhs.midpoint.z != rhs.midpoint.z) return lhs.midpoint.z < rhs.midpoint.z;
    if (lhs.midpoint.y != rhs.midpoint.y) return lhs.midpoint.y < rhs.midpoint.y;
    if (lhs.polygon != rhs.polygon) return lhs.polygon < rhs.polygon;
    return lhs.island < rhs.island;
}

} // namespace

BuildStatus extractBoundaries(
    const dtNavMesh& navMesh,
    const IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    std::vector<Boundary>& output,
    std::string& message) {
    std::unordered_map<BoundaryKey, Boundary, BoundaryKeyHash> boundaries;
    const float cellSize = config.boundaries.effectiveDeduplicationCellSize(
        config.gapDiscovery.maxHorizontalGap);
    const float verticalCellSize = effectiveVerticalCollapseWindow(config);
    for (const Island& island : graph.islands()) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (island.suppressed) {
            continue;
        }
        for (dtPolyRef reference : island.polygons) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
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
                const Vec3 start = fromDetour(&tile->verts[polygon->verts[edge] * 3]);
                const Vec3 end = fromDetour(
                    &tile->verts[polygon->verts[(edge + 1) % polygon->vertCount] * 3]);
                if (!isFinite(start) || !isFinite(end)) {
                    message = "Navmesh boundary contains non-finite coordinates.";
                    return BuildStatus::InvalidNavMesh;
                }
                const Vec3 midpoint = divide(add(start, end), 2.0f);
                const Vec3 direction{end.x - start.x, end.y - start.y, end.z - start.z};
                if (!isFinite(midpoint) || !isFinite(direction)) {
                    message = "Navmesh boundary contains non-finite coordinates.";
                    return BuildStatus::InvalidNavMesh;
                }
                const Boundary boundary{island.id, reference, start, end, midpoint};
                if (config.boundaries.deduplicationEnabled) {
                    const BoundaryKey key{
                        island.id,
                        quantize(midpoint.x, cellSize),
                        quantize(midpoint.y, verticalCellSize),
                        quantize(midpoint.z, cellSize),
                        quantize(direction.x, cellSize),
                        quantize(direction.z, cellSize)};
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
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            output.push_back(entry.second);
        }
    }
    stats.boundaries.deduplicatedCount = output.size();
    std::sort(output.begin(), output.end(), boundaryLess);
    return BuildStatus::Success;
}

BuildStatus selectBoundaryRepresentatives(
    const std::vector<Boundary>& boundaries,
    const IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    std::vector<Boundary>& representatives) {
    std::vector<bool> outboundIslands(graph.islands().size(), true);
    if (config.outboundIslandFilter) {
        for (const Island& island : graph.islands()) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            outboundIslands[island.id] = !island.suppressed && config.outboundIslandFilter(island, graph);
        }
    } else {
        for (const Island& island : graph.islands()) {
            if (island.id < outboundIslands.size()) {
                outboundIslands[island.id] = !island.suppressed;
            }
        }
    }
    std::vector<Boundary> outboundBoundaries;
    outboundBoundaries.reserve(boundaries.size());
    for (const Boundary& boundary : boundaries) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (outboundIslands[boundary.island]) {
            outboundBoundaries.push_back(boundary);
        }
    }
    stats.boundaries.outboundFilteredCount = boundaries.size() - outboundBoundaries.size();
    if (!config.boundaries.representativeReductionEnabled) {
        stats.boundaries.representativeCount = outboundBoundaries.size();
        representatives = std::move(outboundBoundaries);
        return BuildStatus::Success;
    }

    const float cellSize = config.boundaries.effectiveRepresentativeCellSize(
        config.gapDiscovery.maxHorizontalGap);
    const float verticalCellSize = effectiveVerticalCollapseWindow(config);
    struct RankedBoundary {
        Boundary boundary;
        float rank = 0.0f;
    };
    std::unordered_map<BoundaryRepresentativeKey, RankedBoundary, BoundaryRepresentativeKeyHash> bestByCell;
    for (const Boundary& boundary : outboundBoundaries) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        const Vec3 direction{
            boundary.end.x - boundary.start.x,
            boundary.end.y - boundary.start.y,
            boundary.end.z - boundary.start.z};
        const BoundaryRepresentativeKey key{
            boundary.island,
            quantize(boundary.midpoint.x, cellSize),
            quantize(boundary.midpoint.y, verticalCellSize),
            quantize(boundary.midpoint.z, cellSize),
            config.boundaries.representativeDirectionBucket(direction)};
        const BoundaryRepresentativeCandidate candidate{
            boundary.island,
            boundary.polygon,
            boundary.start,
            boundary.end,
            boundary.midpoint};
        float rank = distanceSquared(boundary.start, boundary.end);
        if (config.boundaries.representativeRanker) {
            const float customRank = config.boundaries.representativeRanker(candidate, graph);
            if (std::isfinite(customRank)) {
                rank = customRank;
            }
        }
        const auto existing = bestByCell.find(key);
        if (existing == bestByCell.end() || rank > existing->second.rank) {
            bestByCell[key] = RankedBoundary{boundary, rank};
        }
    }
    std::vector<RankedBoundary> rankedBoundaries;
    rankedBoundaries.reserve(bestByCell.size());
    for (const auto& entry : bestByCell) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        rankedBoundaries.push_back(entry.second);
    }
    if (config.boundaries.maxRepresentativesPerIsland > 0 ||
        config.boundaries.representativeBudgetScale > 0.0f) {
        std::sort(
            rankedBoundaries.begin(),
            rankedBoundaries.end(),
            [](const RankedBoundary& lhs, const RankedBoundary& rhs) {
                if (lhs.boundary.island != rhs.boundary.island) {
                    return lhs.boundary.island < rhs.boundary.island;
                }
                return boundarySpatialLess(lhs.boundary, rhs.boundary);
            });
        representatives.reserve(rankedBoundaries.size());
        for (std::size_t islandBegin = 0; islandBegin < rankedBoundaries.size();) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            const IslandId currentIsland = rankedBoundaries[islandBegin].boundary.island;
            std::size_t islandEnd = islandBegin;
            while (islandEnd < rankedBoundaries.size() &&
                rankedBoundaries[islandEnd].boundary.island == currentIsland) {
                ++islandEnd;
            }

            const std::size_t islandCandidateCount = islandEnd - islandBegin;
            const float massScore = currentIsland < graph.islands().size()
                ? graph.islands()[currentIsland].massScore
                : 0.0f;
            const std::uint32_t budgetForIsland = config.boundaries.representativeBudgetFor(
                massScore,
                config.massAware.enabled,
                islandCandidateCount);
            if (budgetForIsland == 0) {
                islandBegin = islandEnd;
                continue;
            }
            if (budgetForIsland >= islandCandidateCount) {
                for (std::size_t index = islandBegin; index < islandEnd; ++index) {
                    if (cancellationRequested(options)) {
                        return BuildStatus::Cancelled;
                    }
                    representatives.push_back(rankedBoundaries[index].boundary);
                }
                islandBegin = islandEnd;
                continue;
            }

            // Guardrail: large islands must spend sparse scan budget around their boundary,
            // not only on the longest local edges. Each spatial segment keeps its best-ranked
            // representative so mainland-style islands retain geographically distributed exits.
            for (std::uint32_t slot = 0; slot < budgetForIsland; ++slot) {
                if (cancellationRequested(options)) {
                    return BuildStatus::Cancelled;
                }
                const std::size_t segmentBegin =
                    islandBegin + ((static_cast<std::size_t>(slot) * islandCandidateCount) / budgetForIsland);
                const std::size_t segmentEnd =
                    islandBegin + (((static_cast<std::size_t>(slot) + 1U) * islandCandidateCount) / budgetForIsland);
                std::size_t bestIndex = segmentBegin;
                for (std::size_t index = segmentBegin + 1U; index < segmentEnd; ++index) {
                    if (rankedBoundaries[index].rank > rankedBoundaries[bestIndex].rank ||
                        (rankedBoundaries[index].rank == rankedBoundaries[bestIndex].rank &&
                            boundaryLess(rankedBoundaries[index].boundary, rankedBoundaries[bestIndex].boundary))) {
                        bestIndex = index;
                    }
                }
                representatives.push_back(rankedBoundaries[bestIndex].boundary);
            }
            islandBegin = islandEnd;
        }
    } else {
        representatives.reserve(rankedBoundaries.size());
        for (const RankedBoundary& rankedBoundary : rankedBoundaries) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            representatives.push_back(rankedBoundary.boundary);
        }
    }
    std::sort(representatives.begin(), representatives.end(), boundaryLess);
    stats.boundaries.representativeCount = representatives.size();
    stats.boundaries.representativeTrimmedCount = outboundBoundaries.size() - representatives.size();
    return BuildStatus::Success;
}

} // namespace detour_island_graph::detail::discovery
