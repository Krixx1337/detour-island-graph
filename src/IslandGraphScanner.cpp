#include "IslandGraphDiscoveryInternal.h"

#include "VectorMath.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace detour_island_graph::detail::discovery {
namespace {

struct LinkKey {
    IslandId fromIsland = 0;
    IslandId toIsland = 0;
    // Guardrail: preserve distinct vertical layers without turning every tiny height change into a
    // brand new corridor. Large 3D maps need finite height awareness instead of an infinite y merge.
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

struct PairScanKey {
    IslandId fromIsland = 0;
    IslandId toIsland = 0;
    // Guardrail: scan suppression tracks lateral boundary cells so repeated probes of the same
    // corridor at slightly different heights do not explode discovery cost on complex 3D maps.
    SpatialCoordinate midpointX = 0;
    SpatialCoordinate midpointZ = 0;

    bool operator==(const PairScanKey& other) const {
        return fromIsland == other.fromIsland &&
            toIsland == other.toIsland &&
            midpointX == other.midpointX &&
            midpointZ == other.midpointZ;
    }
};

struct PairScanKeyHash {
    std::size_t operator()(const PairScanKey& key) const {
        std::size_t hash = 0;
        hashCombine(hash, key.fromIsland);
        hashCombine(hash, key.toIsland);
        hashCombine(hash, key.midpointX);
        hashCombine(hash, key.midpointZ);
        return hash;
    }
};

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

} // namespace

BuildStatus discoverCandidates(
    dtNavMeshQuery& query,
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    const std::vector<Boundary>& boundaries,
    const std::vector<Boundary>& representatives,
    BuildStats& stats,
    std::vector<Link>& candidates,
    std::string& message) {
    const Clock::time_point discoveryStart = Clock::now();
    std::unordered_map<LinkKey, Link, LinkKeyHash> deduplicated;
    std::unordered_set<PairScanKey, PairScanKeyHash> scannedPairCells;
    std::vector<dtPolyRef> nearby;
    PolygonCollector collector(nearby);
    std::vector<bool> outboundIslands(graph.islands().size(), true);
    const float verticalCellSize = effectiveVerticalCollapseWindow(config);
    const float pairScanCellSize = config.density.pairScanSuppression.effectiveCellSize(
        config.gapDiscovery.maxHorizontalGap);
    dtQueryFilter filter;
    filter.setIncludeFlags(config.query.includeFlags);
    filter.setExcludeFlags(config.query.excludeFlags);
    if (config.outboundIslandFilter) {
        for (const Island& island : graph.islands()) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            outboundIslands[island.id] = config.outboundIslandFilter(island, graph);
        }
    }

    const auto recordCandidate = [&](const Link& link, bool recovery) {
        if (!outboundIslands[link.fromIsland]) {
            return;
        }
        if (!config.density.candidateDeduplication.enabled) {
            if (recovery) {
                ++stats.candidates.shortGapRecoveredCount;
            }
            candidates.push_back(link);
            return;
        }
        const float candidateCellSize =
            config.density.candidateDeduplication.effectiveCellSize(
                link.horizontalDistance,
                config.gapDiscovery.maxHorizontalGap);
        const LinkKey key{
            link.fromIsland,
            link.toIsland,
            quantize(link.start.x, candidateCellSize),
            quantize(link.start.y, verticalCellSize),
            quantize(link.start.z, candidateCellSize),
            quantize(link.end.x, candidateCellSize),
            quantize(link.end.y, verticalCellSize),
            quantize(link.end.z, candidateCellSize)};
        const auto existing = deduplicated.find(key);
        if (existing == deduplicated.end() || isBetterLink(link, existing->second, graph, config)) {
            if (recovery) {
                ++stats.candidates.shortGapRecoveredCount;
            }
            deduplicated[key] = link;
        }
    };

    const auto scan = [&](const std::vector<Boundary>& scanBoundaries, float maxHorizontalGap, bool recovery) {
        const float configuredVerticalExtent =
            (std::max)(config.gapDiscovery.maxVerticalGapUp, config.gapDiscovery.maxVerticalGapDown);
        const float extents[3]{
            maxHorizontalGap,
            recovery ? (std::min)(maxHorizontalGap, configuredVerticalExtent) : configuredVerticalExtent,
            maxHorizontalGap};
        for (const Boundary& boundary : scanBoundaries) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            float center[3];
            toDetour(boundary.midpoint, center);
            nearby.clear();
            ++stats.queries.count;
            if (recovery) {
                ++stats.candidates.shortGapRecoveryQueryCount;
            }
            if (dtStatusFailed(query.queryPolygons(center, extents, &filter, &collector))) {
                message = "dtNavMeshQuery::queryPolygons failed.";
                return BuildStatus::QueryFailed;
            }
            stats.queries.nearbyPolygonCount += nearby.size();
            for (dtPolyRef candidatePolygon : nearby) {
                if (cancellationRequested(options)) {
                    return BuildStatus::Cancelled;
                }
                if (candidatePolygon == 0 || candidatePolygon == boundary.polygon) {
                    continue;
                }
                const auto target = graph.findIslandForPolygon(candidatePolygon);
                if (!target || *target == boundary.island) {
                    continue;
                }
                ++stats.candidates.pairScanCandidateCount;
                if (!recovery && config.density.pairScanSuppression.enabled) {
                    const PairScanKey pairScanKey{
                        boundary.island,
                        *target,
                        quantize(boundary.midpoint.x, pairScanCellSize),
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
                link.end = fromDetour(projected);
                if (!isFinite(link.end)) {
                    message = "Navmesh query returned non-finite coordinates.";
                    return BuildStatus::InvalidNavMesh;
                }
                link.horizontalDistance = horizontalDistance(link.start, link.end);
                link.verticalDistance = link.end.y - link.start.y;
                if (link.horizontalDistance > maxHorizontalGap ||
                    link.verticalDistance > config.gapDiscovery.maxVerticalGapUp ||
                    link.verticalDistance < -config.gapDiscovery.maxVerticalGapDown) {
                    continue;
                }
                recordCandidate(link, recovery);
            }
        }
        return BuildStatus::Success;
    };
    BuildStatus status = scan(representatives, config.gapDiscovery.maxHorizontalGap, false);
    if (status != BuildStatus::Success) {
        return status;
    }
    if (config.density.shortGapRecovery.enabled) {
        status = scan(
            boundaries,
            config.density.shortGapRecovery.effectiveMaxHorizontalGap(config.gapDiscovery.maxHorizontalGap),
            true);
        if (status != BuildStatus::Success) {
            return status;
        }
    }
    if (config.density.candidateDeduplication.enabled) {
        candidates.reserve(deduplicated.size());
        for (const auto& entry : deduplicated) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            candidates.push_back(entry.second);
        }
    }
    stats.candidates.deduplicatedCount = candidates.size();
    stats.timings.linkDiscoveryMs = elapsedMilliseconds(discoveryStart);
    return BuildStatus::Success;
}

} // namespace detour_island_graph::detail::discovery
