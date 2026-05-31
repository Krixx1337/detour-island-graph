#include "IslandGraphBuilderInternal.h"

#include "VectorMath.h"

#include <DetourAlloc.h>
#include <DetourNavMeshQuery.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace detour_island_graph {

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

using SpatialCoordinate = std::int64_t;

struct BoundaryKey {
    IslandId island = 0;
    SpatialCoordinate midpointX = 0;
    SpatialCoordinate midpointY = 0;
    SpatialCoordinate midpointZ = 0;
    SpatialCoordinate directionX = 0;
    SpatialCoordinate directionY = 0;
    SpatialCoordinate directionZ = 0;

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
    SpatialCoordinate startX = 0;
    SpatialCoordinate startY = 0;
    SpatialCoordinate startZ = 0;
    SpatialCoordinate endX = 0;
    SpatialCoordinate endY = 0;
    SpatialCoordinate endZ = 0;

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

struct PairScanKey {
    IslandId fromIsland = 0;
    IslandId toIsland = 0;
    SpatialCoordinate midpointX = 0;
    SpatialCoordinate midpointY = 0;
    SpatialCoordinate midpointZ = 0;

    bool operator==(const PairScanKey& other) const {
        return fromIsland == other.fromIsland &&
            toIsland == other.toIsland &&
            midpointX == other.midpointX &&
            midpointY == other.midpointY &&
            midpointZ == other.midpointZ;
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

struct PairScanKeyHash {
    std::size_t operator()(const PairScanKey& key) const {
        std::size_t hash = 0;
        hashCombine(hash, key.fromIsland);
        hashCombine(hash, key.toIsland);
        hashCombine(hash, key.midpointX);
        hashCombine(hash, key.midpointY);
        hashCombine(hash, key.midpointZ);
        return hash;
    }
};

struct GlobalCellKey {
    SpatialCoordinate x = 0;
    SpatialCoordinate y = 0;
    SpatialCoordinate z = 0;

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

SpatialCoordinate quantize(float value, float cellSize) {
    const double quantized = std::floor(static_cast<double>(value) / static_cast<double>(cellSize));
    if (!std::isfinite(quantized)) {
        return 0;
    }
    if (quantized <= static_cast<double>((std::numeric_limits<SpatialCoordinate>::min)())) {
        return (std::numeric_limits<SpatialCoordinate>::min)();
    }
    if (quantized >= static_cast<double>((std::numeric_limits<SpatialCoordinate>::max)())) {
        return (std::numeric_limits<SpatialCoordinate>::max)();
    }
    return static_cast<SpatialCoordinate>(quantized);
}

class PolygonCollector final : public dtPolyQuery {
public:
    explicit PolygonCollector(std::vector<dtPolyRef>& polygons)
        : m_polygons(polygons) {}

    void process(const dtMeshTile*, dtPoly**, dtPolyRef* refs, int count) override {
        m_polygons.insert(m_polygons.end(), refs, refs + count);
    }

private:
    std::vector<dtPolyRef>& m_polygons;
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

} // namespace

namespace detail {

bool validate(const BuildConfig& config, std::string& message) {
    const MassAwareTuning& massAware = config.massAware;
    const DensityTuning& density = config.density;
    const auto require = [&](bool condition, const char* error) {
        if (!condition) {
            message = error;
        }
        return condition;
    };

    if (!require(
            std::isfinite(config.gapDiscovery.maxHorizontalGap) &&
                config.gapDiscovery.maxHorizontalGap > 0.0f,
            "gapDiscovery.maxHorizontalGap must be finite and greater than zero.")) return false;
    if (!require(
            std::isfinite(config.gapDiscovery.maxVerticalGapUp) &&
                config.gapDiscovery.maxVerticalGapUp >= 0.0f,
            "gapDiscovery.maxVerticalGapUp must be finite and non-negative.")) return false;
    if (!require(
            std::isfinite(config.gapDiscovery.maxVerticalGapDown) &&
                config.gapDiscovery.maxVerticalGapDown >= 0.0f,
            "gapDiscovery.maxVerticalGapDown must be finite and non-negative.")) return false;
    if (!require(
            !config.boundaries.deduplicationEnabled ||
                (std::isfinite(config.boundaries.deduplicationCellSize) &&
                 config.boundaries.deduplicationCellSize >= 0.0f &&
                 std::isfinite(config.boundaries.deduplicationCellSizeRatio) &&
                 config.boundaries.deduplicationCellSizeRatio > 0.0f),
            "Enabled boundary deduplication requires a non-negative finite cell size and a positive finite cell-size ratio.")) return false;
    if (!require(
            !config.boundaries.representativeReductionEnabled ||
                (std::isfinite(config.boundaries.representativeCellSize) &&
                 config.boundaries.representativeCellSize >= 0.0f &&
                 std::isfinite(config.boundaries.representativeCellSizeRatio) &&
                 config.boundaries.representativeCellSizeRatio > 0.0f),
            "Enabled boundary representative reduction requires a non-negative finite cell size and a positive finite cell-size ratio.")) return false;
    if (!require(config.query.maxNodes > 0, "query.maxNodes must be greater than zero.")) return false;
    if (!require(
            std::isfinite(massAware.normalizationPercentile) &&
                massAware.normalizationPercentile > 0.0f &&
                massAware.normalizationPercentile <= 1.0f,
            "massAware.normalizationPercentile must be finite and in the range (0, 1].")) return false;
    if (!require(
            std::isfinite(massAware.targetPreference) && massAware.targetPreference >= 0.0f,
            "massAware.targetPreference must be finite and non-negative.")) return false;
    if (!require(
            std::isfinite(massAware.lowMassPruneRadiusScale) && massAware.lowMassPruneRadiusScale > 0.0f,
            "massAware.lowMassPruneRadiusScale must be finite and greater than zero.")) return false;
    if (!require(
            std::isfinite(massAware.highMassPruneRadiusScale) && massAware.highMassPruneRadiusScale > 0.0f,
            "massAware.highMassPruneRadiusScale must be finite and greater than zero.")) return false;
    if (!require(
            !density.pairScanSuppression.enabled ||
                (std::isfinite(density.pairScanSuppression.cellSize) &&
                 density.pairScanSuppression.cellSize >= 0.0f &&
                 std::isfinite(density.pairScanSuppression.cellSizeRatio) &&
                 density.pairScanSuppression.cellSizeRatio > 0.0f),
            "Enabled pair-scan suppression requires a non-negative finite cell size and a positive finite cell-size ratio.")) return false;
    if (!require(
            !density.candidateDeduplication.enabled ||
                (std::isfinite(density.candidateDeduplication.cellSize) &&
                 density.candidateDeduplication.cellSize >= 0.0f &&
                 std::isfinite(density.candidateDeduplication.cellSizeRatio) &&
                 density.candidateDeduplication.cellSizeRatio > 0.0f),
            "Enabled candidate deduplication requires a non-negative finite cell size and a positive finite cell-size ratio.")) return false;
    if (!require(
            !density.localPruning.enabled ||
                (std::isfinite(density.localPruning.baseRadius) &&
                 density.localPruning.baseRadius >= 0.0f &&
                 std::isfinite(density.localPruning.baseRadiusRatio) &&
                 density.localPruning.baseRadiusRatio > 0.0f),
            "Enabled local pruning requires a non-negative finite base radius and a positive finite base-radius ratio.")) return false;
    if (!require(
            !density.localPruning.enabled ||
                !density.localPruning.enableDistanceScaling ||
                (std::isfinite(density.localPruning.distanceScale) &&
                 density.localPruning.distanceScale >= 0.0f &&
                 std::isfinite(density.localPruning.maxRadiusScale) &&
                 density.localPruning.maxRadiusScale >= 1.0f),
            "Enabled local-pruning distance scaling requires a non-negative finite distance scale and a finite maximum radius scale of at least one.")) return false;
    if (!require(
            !density.globalPruning.enabled ||
                (std::isfinite(density.globalPruning.cellSize) &&
                 density.globalPruning.cellSize >= 0.0f &&
                 std::isfinite(density.globalPruning.cellSizeRatio) &&
                 density.globalPruning.cellSizeRatio > 0.0f),
            "Enabled global pruning requires a non-negative finite cell size and a positive finite cell-size ratio.")) return false;
    if (!require(
            !density.spannerPruning.enabled ||
                (std::isfinite(density.spannerPruning.pathRatio) &&
                 density.spannerPruning.pathRatio >= 1.0f &&
                 std::isfinite(density.spannerPruning.verticalWeight) &&
                 density.spannerPruning.verticalWeight >= 0.0f),
            "Enabled spanner pruning requires a finite path ratio of at least one and a non-negative finite vertical weight.")) return false;
    return true;
}

} // namespace detail

namespace {

std::vector<Boundary> extractBoundaries(
    const dtNavMesh& navMesh,
    const IslandGraph& graph,
    const BuildConfig& config,
    BuildStats& stats) {
    std::unordered_map<BoundaryKey, Boundary, BoundaryKeyHash> boundaries;
    std::vector<Boundary> output;
    const float cellSize = config.boundaries.effectiveDeduplicationCellSize(
        config.gapDiscovery.maxHorizontalGap);
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
                    const BoundaryKey key{
                        island.id,
                        quantize(midpoint.x, cellSize),
                        quantize(midpoint.y, cellSize),
                        quantize(midpoint.z, cellSize),
                        quantize(direction.x, cellSize),
                        quantize(direction.y, cellSize),
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

std::vector<Boundary> selectBoundaryRepresentatives(
    const std::vector<Boundary>& boundaries,
    const BuildConfig& config,
    BuildStats& stats) {
    if (!config.boundaries.representativeReductionEnabled) {
        stats.boundaries.representativeCount = boundaries.size();
        return boundaries;
    }

    const float cellSize = config.boundaries.effectiveRepresentativeCellSize(
        config.gapDiscovery.maxHorizontalGap);
    std::unordered_set<BoundaryKey, BoundaryKeyHash> occupiedCells;
    std::vector<Boundary> representatives;
    representatives.reserve(boundaries.size());
    for (const Boundary& boundary : boundaries) {
        const Vec3 direction{
            boundary.end.x - boundary.start.x,
            boundary.end.y - boundary.start.y,
            boundary.end.z - boundary.start.z};
        const BoundaryKey key{
            boundary.island,
            quantize(boundary.midpoint.x, cellSize),
            quantize(boundary.midpoint.y, cellSize),
            quantize(boundary.midpoint.z, cellSize),
            quantize(direction.x, cellSize),
            quantize(direction.y, cellSize),
            quantize(direction.z, cellSize)};
        if (occupiedCells.emplace(key).second) {
            representatives.push_back(boundary);
        }
    }
    stats.boundaries.representativeCount = representatives.size();
    stats.boundaries.representativeTrimmedCount = boundaries.size() - representatives.size();
    return representatives;
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

BuildStatus discoverCandidates(
    dtNavMeshQuery& query,
    IslandGraph& graph,
    const BuildConfig& config,
    const std::vector<Boundary>& representatives,
    BuildStats& stats,
    std::vector<Link>& candidates,
    std::string& message) {
    const Clock::time_point discoveryStart = Clock::now();
    std::unordered_map<LinkKey, Link, LinkKeyHash> deduplicated;
    std::unordered_set<PairScanKey, PairScanKeyHash> scannedPairCells;
    std::vector<dtPolyRef> nearby;
    PolygonCollector collector(nearby);
    const float candidateCellSize = config.density.candidateDeduplication.effectiveCellSize(
        config.gapDiscovery.maxHorizontalGap);
    const float pairScanCellSize = config.density.pairScanSuppression.effectiveCellSize(
        config.gapDiscovery.maxHorizontalGap);
    const float extents[3]{
        config.gapDiscovery.maxHorizontalGap,
        (std::max)(config.gapDiscovery.maxVerticalGapUp, config.gapDiscovery.maxVerticalGapDown),
        config.gapDiscovery.maxHorizontalGap};
    const dtQueryFilter filter;

    for (const Boundary& boundary : representatives) {
        float center[3];
        detail::toDetour(boundary.midpoint, center);
        nearby.clear();
        ++stats.queries.count;
        if (dtStatusFailed(query.queryPolygons(
                center,
                extents,
                &filter,
                &collector))) {
            message = "dtNavMeshQuery::queryPolygons failed.";
            return BuildStatus::QueryFailed;
        }
        stats.queries.nearbyPolygonCount += nearby.size();
        for (dtPolyRef candidatePolygon : nearby) {
            if (candidatePolygon == 0 || candidatePolygon == boundary.polygon) {
                continue;
            }
            const auto target = graph.findIslandForPolygon(candidatePolygon);
            if (!target || *target == boundary.island) {
                continue;
            }
            ++stats.candidates.pairScanCandidateCount;
            if (config.density.pairScanSuppression.enabled) {
                const PairScanKey pairScanKey{
                    boundary.island,
                    *target,
                    quantize(boundary.midpoint.x, pairScanCellSize),
                    quantize(boundary.midpoint.y, pairScanCellSize),
                    quantize(boundary.midpoint.z, pairScanCellSize)};
                if (!scannedPairCells.insert(pairScanKey).second) {
                    ++stats.candidates.pairScanSuppressedCount;
                    continue;
                }
            }
            float projected[3];
            bool overPolygon = false;
            ++stats.candidates.closestPointQueryCount;
            if (dtStatusFailed(query.closestPointOnPoly(candidatePolygon, center, projected, &overPolygon))) {
                ++stats.candidates.closestPointFailureCount;
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
            const LinkKey key{
                link.fromIsland,
                link.toIsland,
                quantize(link.start.x, candidateCellSize),
                quantize(link.start.y, candidateCellSize),
                quantize(link.start.z, candidateCellSize),
                quantize(link.end.x, candidateCellSize),
                quantize(link.end.y, candidateCellSize),
                quantize(link.end.z, candidateCellSize)};
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
    return BuildStatus::Success;
}

void pruneCandidates(
    IslandGraph& graph,
    const BuildConfig& config,
    BuildStats& stats,
    std::vector<Link>& candidates) {
    const Clock::time_point pruningStart = Clock::now();
    std::sort(candidates.begin(), candidates.end(), [&](const Link& lhs, const Link& rhs) {
        return isBetterLink(lhs, rhs, graph, config);
    });

    std::unordered_set<GlobalCellKey, GlobalCellKeyHash> occupiedGlobalCells;
    std::unordered_map<IslandPair, std::vector<Link>, IslandPairHash> acceptedByPair;
    const float globalCellSize = config.density.globalPruning.effectiveCellSize(
        config.gapDiscovery.maxHorizontalGap);
    auto& islands = detail::IslandGraphAccess::islands(graph);
    for (const Link& candidate : candidates) {
        if (config.density.globalPruning.enabled) {
            const GlobalCellKey startCell{
                quantize(candidate.start.x, globalCellSize),
                quantize(candidate.start.y, globalCellSize),
                quantize(candidate.start.z, globalCellSize)
            };
            const GlobalCellKey endCell{
                quantize(candidate.end.x, globalCellSize),
                quantize(candidate.end.y, globalCellSize),
                quantize(candidate.end.z, globalCellSize)
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
            const float radiusSquared = radius * radius;
            duplicate = std::any_of(accepted.begin(), accepted.end(), [&](const Link& existing) {
                return detail::distanceSquared(candidate.start, existing.start) <= radiusSquared &&
                    detail::distanceSquared(candidate.end, existing.end) <= radiusSquared;
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
                const GlobalCellKey startCell{
                    quantize(candidate.start.x, globalCellSize),
                    quantize(candidate.start.y, globalCellSize),
                    quantize(candidate.start.z, globalCellSize)
                };
                const GlobalCellKey endCell{
                    quantize(candidate.end.x, globalCellSize),
                    quantize(candidate.end.y, globalCellSize),
                    quantize(candidate.end.z, globalCellSize)
                };
                occupiedGlobalCells.insert(startCell);
                occupiedGlobalCells.insert(endCell);
            }
        }
    }
    stats.timings.pruningMs = elapsedMilliseconds(pruningStart);
}

} // namespace

namespace detail {

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
    const std::vector<Boundary> representatives = selectBoundaryRepresentatives(boundaries, config, stats);
    stats.timings.boundaryExtractionMs = elapsedMilliseconds(boundaryStart);

    std::vector<Link> candidates;
    const BuildStatus discoveryStatus =
        discoverCandidates(*query, graph, config, representatives, stats, candidates, message);
    if (discoveryStatus != BuildStatus::Success) {
        return discoveryStatus;
    }
    pruneCandidates(graph, config, stats, candidates);
    return BuildStatus::Success;
}

} // namespace detail

} // namespace detour_island_graph
