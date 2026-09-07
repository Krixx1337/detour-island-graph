#include "IslandGraphDiscoveryInternal.h"

#include "VectorMath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

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

BuildStatus assembleSamplingTopIslands(
    const IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    const std::vector<Boundary>& boundaries,
    const std::vector<Boundary>& representatives) {
    std::vector<IslandId> ranked;
    ranked.reserve(graph.islands().size());
    for (const Island& island : graph.islands()) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        ranked.push_back(island.id);
    }
    std::sort(ranked.begin(), ranked.end(), [&](IslandId lhs, IslandId rhs) {
        const std::size_t lhsPolygons = lhs < graph.islands().size()
            ? graph.islands()[lhs].polygons.size()
            : 0;
        const std::size_t rhsPolygons = rhs < graph.islands().size()
            ? graph.islands()[rhs].polygons.size()
            : 0;
        if (lhsPolygons != rhsPolygons) {
            return lhsPolygons > rhsPolygons;
        }
        return lhs < rhs;
    });
    const auto countAt = [&](const std::vector<std::size_t>& counts, IslandId island) {
        return island < counts.size() ? counts[island] : 0;
    };
    const auto lengthAt = [&](const std::vector<double>& lengths, IslandId island) {
        return island < lengths.size() ? lengths[island] : 0.0;
    };
    stats.boundaries.samplingTopIslands.clear();
    const std::size_t shown = (std::min)(ranked.size(), BoundaryStats::kMaxSamplingIslands);
    // Guardrail: spatial coverage is measured only for the reported rows. Restricting the
    // quadratic nearest-representative search to the largest islands keeps diagnostic
    // work proportional to what the report can actually show.
    std::unordered_set<IslandId> sampledIslands;
    for (std::size_t index = 0; index < shown; ++index) {
        sampledIslands.insert(ranked[index]);
    }
    std::unordered_map<IslandId, std::vector<Vec3>> sampleMidpoints;
    std::unordered_map<IslandId, std::vector<Vec3>> representativeMidpoints;
    for (const Boundary& boundary : boundaries) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (sampledIslands.find(boundary.island) != sampledIslands.end()) {
            sampleMidpoints[boundary.island].push_back(boundary.midpoint);
        }
    }
    for (const Boundary& representative : representatives) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (sampledIslands.find(representative.island) != sampledIslands.end()) {
            representativeMidpoints[representative.island].push_back(representative.midpoint);
        }
    }
    const float maxHorizontalGap = config.gapDiscovery.maxHorizontalGap;
    const float maxVerticalReach = (std::max)(
        config.gapDiscovery.maxVerticalGapUp,
        config.gapDiscovery.maxVerticalGapDown);
    for (std::size_t index = 0; index < shown; ++index) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        const IslandId island = ranked[index];
        IslandBoundarySampling row;
        row.island = island;
        row.polygonCount = island < graph.islands().size()
            ? graph.islands()[island].polygons.size()
            : 0;
        row.rawBoundaries = countAt(stats.boundaries.rawCountByIsland, island);
        row.deduplicatedBoundaries = countAt(stats.boundaries.deduplicatedCountByIsland, island);
        row.reducedRepresentatives = countAt(stats.boundaries.reducedCountByIsland, island);
        row.scannedRepresentatives = countAt(stats.boundaries.representativeCountByIsland, island);
        row.rawLength = lengthAt(stats.boundaries.rawLengthByIsland, island);
        row.scannedLength = lengthAt(stats.boundaries.scannedLengthByIsland, island);
        const auto samplesIt = sampleMidpoints.find(island);
        const auto repsIt = representativeMidpoints.find(island);
        static const std::vector<Vec3> emptyMidpoints;
        const std::vector<Vec3>& samples =
            samplesIt != sampleMidpoints.end() ? samplesIt->second : emptyMidpoints;
        const std::vector<Vec3>& reps =
            repsIt != representativeMidpoints.end() ? repsIt->second : emptyMidpoints;
        if (!samples.empty() && !reps.empty()) {
            std::vector<double> nearestDistances;
            nearestDistances.reserve(samples.size());
            std::size_t uncoveredSamples = 0;
            for (const Vec3& sample : samples) {
                if (cancellationRequested(options)) {
                    return BuildStatus::Cancelled;
                }
                double best = (std::numeric_limits<double>::max)();
                bool covered = false;
                for (const Vec3& rep : reps) {
                    const double dx = static_cast<double>(sample.x) - rep.x;
                    const double dz = static_cast<double>(sample.z) - rep.z;
                    const double dy = static_cast<double>(sample.y) - rep.y;
                    const double horizontal = std::hypot(dx, dz);
                    if (horizontal <= maxHorizontalGap && std::fabs(dy) <= maxVerticalReach) {
                        covered = true;
                    }
                    const double dist = std::sqrt(horizontal * horizontal + dy * dy);
                    if (dist < best) {
                        best = dist;
                    }
                }
                if (!covered) {
                    ++uncoveredSamples;
                }
                nearestDistances.push_back(best);
            }
            std::sort(nearestDistances.begin(), nearestDistances.end());
            row.nearestRepresentativeP95 =
                nearestDistances[((nearestDistances.size() - 1) * 95U) / 100U];
            row.nearestRepresentativeMax = nearestDistances.back();
            row.uncoveredFraction = static_cast<double>(uncoveredSamples) /
                static_cast<double>(nearestDistances.size());
        } else if (!samples.empty()) {
            row.uncoveredFraction = 1.0;
        }
        stats.boundaries.samplingTopIslands.push_back(row);
    }
    return BuildStatus::Success;
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
    stats.boundaries.rawCountByIsland.assign(graph.islands().size(), 0);
    stats.boundaries.rawLengthByIsland.assign(graph.islands().size(), 0.0);
    stats.boundaries.deduplicatedCountByIsland.assign(graph.islands().size(), 0);
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
                if (island.id < stats.boundaries.rawCountByIsland.size()) {
                    ++stats.boundaries.rawCountByIsland[island.id];
                    stats.boundaries.rawLengthByIsland[island.id] +=
                        static_cast<double>(distance(start, end));
                }
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
    for (const Boundary& boundary : output) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (boundary.island < stats.boundaries.deduplicatedCountByIsland.size()) {
            ++stats.boundaries.deduplicatedCountByIsland[boundary.island];
        }
    }
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
    stats.boundaries.representativeCountByIsland.assign(graph.islands().size(), 0);
    stats.boundaries.reducedCountByIsland.assign(graph.islands().size(), 0);
    stats.boundaries.scannedLengthByIsland.assign(graph.islands().size(), 0.0);
    if (!config.boundaries.representativeReductionEnabled) {
        stats.boundaries.representativeCount = outboundBoundaries.size();
        for (const Boundary& boundary : outboundBoundaries) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            if (boundary.island < stats.boundaries.representativeCountByIsland.size()) {
                ++stats.boundaries.representativeCountByIsland[boundary.island];
                ++stats.boundaries.reducedCountByIsland[boundary.island];
                stats.boundaries.scannedLengthByIsland[boundary.island] +=
                    static_cast<double>(distance(boundary.start, boundary.end));
            }
        }
        representatives = std::move(outboundBoundaries);
        // Without reduction every outbound boundary is scanned, so the sample set and the
        // representative set coincide. (outboundBoundaries was moved from above.)
        return assembleSamplingTopIslands(graph, config, options, stats, representatives, representatives);
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
    for (const RankedBoundary& rankedBoundary : rankedBoundaries) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (rankedBoundary.boundary.island < stats.boundaries.reducedCountByIsland.size()) {
            ++stats.boundaries.reducedCountByIsland[rankedBoundary.boundary.island];
        }
    }
    for (const Boundary& representative : representatives) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (representative.island < stats.boundaries.representativeCountByIsland.size()) {
            ++stats.boundaries.representativeCountByIsland[representative.island];
            stats.boundaries.scannedLengthByIsland[representative.island] +=
                static_cast<double>(distance(representative.start, representative.end));
        }
    }
    return assembleSamplingTopIslands(graph, config, options, stats, boundaries, representatives);
}

} // namespace detour_island_graph::detail::discovery
