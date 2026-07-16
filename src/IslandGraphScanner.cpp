#include "IslandGraphDiscoveryInternal.h"

#include "VectorMath.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace detour_island_graph::detail::discovery {
namespace {

struct LinkKey {
    IslandId islandA = 0;
    IslandId islandB = 0;
    // Guardrail: preserve distinct vertical layers without turning every tiny height change into a
    // brand new corridor. Large 3D maps need finite height awareness instead of an infinite y merge.
    SpatialCoordinate startX = 0;
    SpatialCoordinate startY = 0;
    SpatialCoordinate startZ = 0;
    SpatialCoordinate endX = 0;
    SpatialCoordinate endY = 0;
    SpatialCoordinate endZ = 0;

    bool operator==(const LinkKey& other) const {
        return islandA == other.islandA &&
            islandB == other.islandB &&
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
        hashCombine(hash, key.islandA);
        hashCombine(hash, key.islandB);
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
    IslandId islandA = 0;
    IslandId islandB = 0;
    // Guardrail: scan suppression uses a 3D cell so sparse builds do not erase stacked routes
    // that share lateral coordinates but live on different vertical layers.
    SpatialCoordinate midpointX = 0;
    SpatialCoordinate midpointY = 0;
    SpatialCoordinate midpointZ = 0;

    bool operator==(const PairScanKey& other) const {
        return islandA == other.islandA &&
            islandB == other.islandB &&
            midpointX == other.midpointX &&
            midpointY == other.midpointY &&
            midpointZ == other.midpointZ;
    }
};

struct PairScanKeyHash {
    std::size_t operator()(const PairScanKey& key) const {
        std::size_t hash = 0;
        hashCombine(hash, key.islandA);
        hashCombine(hash, key.islandB);
        hashCombine(hash, key.midpointX);
        hashCombine(hash, key.midpointY);
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
    std::vector<std::pair<IslandId, dtPolyRef>> targetPolygons;
    PolygonCollector collector(nearby);
    std::vector<bool> outboundIslands(graph.islands().size(), true);
    const bool symmetricCapabilities = hasSymmetricVerticalCapabilities(config);
    const bool sphericalTraversalEnvelope = usesSphericalTraversalEnvelope(config);
    const float pairScanCellSize = config.density.pairScanSuppression.effectiveCellSize(
        maxTraversalExtent(config));
    dtQueryFilter filter;
    filter.setIncludeFlags(config.query.includeFlags);
    filter.setExcludeFlags(config.query.excludeFlags);
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
            config.density.candidateDeduplication.effectiveCellSize(maxTraversalExtent(config));
        const bool normalizeOrder = symmetricCapabilities && link.toIsland < link.fromIsland;
        const LinkKey key{
            normalizeOrder ? link.toIsland : link.fromIsland,
            normalizeOrder ? link.fromIsland : link.toIsland,
            quantize(normalizeOrder ? link.end.x : link.start.x, candidateCellSize),
            quantize(normalizeOrder ? link.end.y : link.start.y, candidateCellSize),
            quantize(normalizeOrder ? link.end.z : link.start.z, candidateCellSize),
            quantize(normalizeOrder ? link.start.x : link.end.x, candidateCellSize),
            quantize(normalizeOrder ? link.start.y : link.end.y, candidateCellSize),
            quantize(normalizeOrder ? link.start.z : link.end.z, candidateCellSize)};
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
            const auto evaluateCandidate = [&](
                                               dtPolyRef candidatePolygon,
                                               IslandId target,
                                               Link& link,
                                               bool& accepted) {
                accepted = false;
                float projected[3];
                bool overPolygon = false;
                ++stats.candidates.closestPointQueryCount;
                if (dtStatusFailed(query.closestPointOnPoly(candidatePolygon, center, projected, &overPolygon))) {
                    ++stats.candidates.closestPointFailureCount;
                    return BuildStatus::Success;
                }
                ++stats.candidates.projectedCount;
                (void)overPolygon;
                link.fromIsland = boundary.island;
                link.toIsland = target;
                link.start = boundary.midpoint;
                link.end = fromDetour(projected);
                if (!isFinite(link.end)) {
                    message = "Navmesh query returned non-finite coordinates.";
                    return BuildStatus::InvalidNavMesh;
                }
                link.horizontalDistance = horizontalDistance(link.start, link.end);
                link.verticalDistance = link.end.y - link.start.y;
                if (sphericalTraversalEnvelope && distance(link.start, link.end) > maxTraversalExtent(config)) {
                    return BuildStatus::Success;
                }
                if (!sphericalTraversalEnvelope &&
                    (link.horizontalDistance > maxHorizontalGap ||
                     link.verticalDistance > config.gapDiscovery.maxVerticalGapUp ||
                     link.verticalDistance < -config.gapDiscovery.maxVerticalGapDown)) {
                    return BuildStatus::Success;
                }
                accepted = true;
                return BuildStatus::Success;
            };

            targetPolygons.clear();
            targetPolygons.reserve(nearby.size());
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
                if (*target >= graph.islands().size() || graph.islands()[*target].suppressed) {
                    continue;
                }
                if (!recovery && config.density.pairScanSuppression.enabled) {
                    targetPolygons.push_back({*target, candidatePolygon});
                    continue;
                }
                Link link;
                bool accepted = false;
                ++stats.candidates.pairScanCandidateCount;
                const BuildStatus evaluationStatus =
                    evaluateCandidate(candidatePolygon, *target, link, accepted);
                if (evaluationStatus != BuildStatus::Success) {
                    return evaluationStatus;
                }
                if (accepted) {
                    recordCandidate(link, recovery);
                }
            }

            std::sort(targetPolygons.begin(), targetPolygons.end());
            for (std::size_t targetBegin = 0; targetBegin < targetPolygons.size();) {
                if (cancellationRequested(options)) {
                    return BuildStatus::Cancelled;
                }
                const IslandId target = targetPolygons[targetBegin].first;
                std::size_t targetEnd = targetBegin + 1;
                while (targetEnd < targetPolygons.size() &&
                    targetPolygons[targetEnd].first == target) {
                    ++targetEnd;
                }
                const bool normalizePair = symmetricCapabilities && target < boundary.island;
                const PairScanKey pairScanKey{
                    normalizePair ? target : boundary.island,
                    normalizePair ? boundary.island : target,
                    quantize(boundary.midpoint.x, pairScanCellSize),
                    quantize(boundary.midpoint.y, pairScanCellSize),
                    quantize(boundary.midpoint.z, pairScanCellSize)};
                const auto committed = scannedPairCells.find(pairScanKey);
                if (committed != scannedPairCells.end()) {
                    ++stats.candidates.pairScanSuppressedCount;
                    targetBegin = targetEnd;
                    continue;
                }

                ++stats.candidates.pairScanCandidateCount;
                std::optional<Link> bestLink;
                for (std::size_t targetIndex = targetBegin; targetIndex < targetEnd; ++targetIndex) {
                    if (cancellationRequested(options)) {
                        return BuildStatus::Cancelled;
                    }
                    const dtPolyRef candidatePolygon = targetPolygons[targetIndex].second;
                    Link link;
                    bool accepted = false;
                    const BuildStatus evaluationStatus =
                        evaluateCandidate(candidatePolygon, target, link, accepted);
                    if (evaluationStatus != BuildStatus::Success) {
                        return evaluationStatus;
                    }
                    if (accepted &&
                        (!bestLink.has_value() || isBetterLink(link, *bestLink, graph, config))) {
                        bestLink = link;
                    }
                }
                if (bestLink.has_value()) {
                    // Suppression is a committed-work record: failed projections never consume it.
                    scannedPairCells.insert(pairScanKey);
                    recordCandidate(*bestLink, false);
                }
                targetBegin = targetEnd;
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
