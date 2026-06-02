#include "IslandGraphDiscoveryInternal.h"

#include "VectorMath.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace detour_island_graph::detail::discovery {
namespace {

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

float linkDistance(const Link& link) {
    return distance(link.start, link.end);
}

float rankScore(const Link& link, const IslandGraph& graph, const BuildConfig& config) {
    if (config.linkRanker) {
        const float customRank = config.linkRanker(link, graph);
        if (std::isfinite(customRank)) {
            return customRank;
        }
    }
    const float preference = config.massAware.enabled
        ? config.massAware.targetPreferenceFor(
            graph.islands()[link.toIsland].massScore,
            config.gapDiscovery.maxHorizontalGap)
        : 0.0f;
    return linkDistance(link) - preference;
}

float linkImportance(const Link& link, const IslandGraph& graph) {
    if (link.fromIsland >= graph.islands().size() || link.toIsland >= graph.islands().size()) {
        return 0.0f;
    }
    // Guardrail: when nearby source corridors compete, rank by combined mass first so pruning
    // removes tiny-island fan-out before it deletes the main traversal spine.
    return graph.islands()[link.fromIsland].massScore * graph.islands()[link.toIsland].massScore;
}

float pruneRadius(const Link& link, const IslandGraph& graph, const BuildConfig& config) {
    float scale = 1.0f;
    if (config.massAware.enabled) {
        const float targetMass = graph.islands()[link.toIsland].massScore;
        scale *= config.massAware.pruneRadiusScaleFor(targetMass);
    }
    if (config.density.localPruning.enableDistanceScaling) {
        scale *= config.density.localPruning.pruneRadiusScaleFor(
            link.horizontalDistance,
            config.gapDiscovery.maxHorizontalGap);
    }
    return config.density.localPruning.effectiveBaseRadius(config.gapDiscovery.maxHorizontalGap) * scale;
}

bool withinLocalPruningWindow(
    const Link& candidate,
    const Link& existing,
    float horizontalRadiusSquared,
    float verticalWindow) {
    const float startHorizontalDistance = horizontalDistance(candidate.start, existing.start);
    const float endHorizontalDistance = horizontalDistance(candidate.end, existing.end);
    return (startHorizontalDistance * startHorizontalDistance) <= horizontalRadiusSquared &&
        std::abs(candidate.start.y - existing.start.y) <= verticalWindow &&
        (endHorizontalDistance * endHorizontalDistance) <= horizontalRadiusSquared &&
        std::abs(candidate.end.y - existing.end.y) <= verticalWindow;
}

bool hasAcceptableIndirectRoute(
    const Link& candidate,
    const std::vector<std::vector<Link>>& acceptedOutgoing,
    float pathRatio,
    float verticalWeight) {
    const auto& outgoing = acceptedOutgoing[candidate.fromIsland];
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
        const auto& secondOutgoing = acceptedOutgoing[firstHop.toIsland];
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

} // namespace

bool isBetterLink(const Link& lhs, const Link& rhs, const IslandGraph& graph, const BuildConfig& config) {
    if (config.massAware.enabled) {
        const float lhsImportance = linkImportance(lhs, graph);
        const float rhsImportance = linkImportance(rhs, graph);
        if (std::abs(lhsImportance - rhsImportance) > 0.001f) {
            return lhsImportance > rhsImportance;
        }
    }
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

BuildStatus pruneCandidates(
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    std::vector<Link>& candidates) {
    const Clock::time_point pruningStart = Clock::now();
    if (cancellationRequested(options)) {
        return BuildStatus::Cancelled;
    }
    std::sort(candidates.begin(), candidates.end(), [&](const Link& lhs, const Link& rhs) {
        return isBetterLink(lhs, rhs, graph, config);
    });
    if (cancellationRequested(options)) {
        return BuildStatus::Cancelled;
    }

    auto& islands = IslandGraphAccess::islands(graph);
    auto& edges = IslandGraphAccess::edges(graph);
    const bool symmetricCapabilities = hasSymmetricTraversalCapabilities(config);
    edges.clear();
    for (Island& island : islands) {
        island.edgeIndices.clear();
    }

    std::vector<Vec3> occupiedGlobalPoints;
    std::unordered_map<GlobalCellKey, std::vector<Vec3>, GlobalCellKeyHash> occupiedGlobalCells;
    if (config.density.globalPruning.enabled) {
        occupiedGlobalPoints.reserve(candidates.size() * 2U);
        occupiedGlobalCells.reserve(candidates.size() * 2U);
    }
    const float globalBaseRadius = config.density.globalPruning.enabled
        ? (config.density.globalPruning.radius > 0.0f
            ? config.density.globalPruning.radius
            : config.gapDiscovery.maxHorizontalGap * (std::min)(
                config.density.globalPruning.nearRadiusRatio,
                config.density.globalPruning.farRadiusRatio))
        : 1.0f;
    const auto isNearOccupiedGlobalPoint = [&](const Vec3& candidatePoint, float radiusSquared) {
        return std::any_of(
            occupiedGlobalPoints.begin(),
            occupiedGlobalPoints.end(),
            [&](const Vec3& point) {
                return distanceSquared(candidatePoint, point) <= radiusSquared;
            });
    };
    const auto isOccupiedGlobalPoint = [&](const Vec3& candidatePoint, float radius) {
        const float radiusSquared = radius * radius;
        const float cellRangeFloat = std::ceil(radius / globalBaseRadius);
        if (!std::isfinite(cellRangeFloat) || cellRangeFloat > 4.0f) {
            return isNearOccupiedGlobalPoint(candidatePoint, radiusSquared);
        }
        const int cellRange = static_cast<int>(cellRangeFloat);
        const GlobalCellKey center{
            quantize(candidatePoint.x, globalBaseRadius),
            quantize(candidatePoint.y, globalBaseRadius),
            quantize(candidatePoint.z, globalBaseRadius)};
        for (int x = -cellRange; x <= cellRange; ++x) {
            for (int y = -cellRange; y <= cellRange; ++y) {
                for (int z = -cellRange; z <= cellRange; ++z) {
                    const auto cell = occupiedGlobalCells.find({
                        offsetCoordinate(center.x, x),
                        offsetCoordinate(center.y, y),
                        offsetCoordinate(center.z, z)});
                    if (cell != occupiedGlobalCells.end() &&
                        std::any_of(cell->second.begin(), cell->second.end(), [&](const Vec3& point) {
                            return distanceSquared(candidatePoint, point) <= radiusSquared;
                        })) {
                        return true;
                    }
                }
            }
        }
        return false;
    };
    const auto occupyGlobalPoint = [&](const Vec3& point) {
        occupiedGlobalPoints.push_back(point);
        occupiedGlobalCells[{
            quantize(point.x, globalBaseRadius),
            quantize(point.y, globalBaseRadius),
            quantize(point.z, globalBaseRadius)}].push_back(point);
    };
    // Guardrail: locality ownership lives at the source island, not the (source,target) pair.
    // Complex 3D maps can fragment one real exit into many tiny neighboring targets; pruning only
    // per pair keeps the full fan-out and recreates link bloat even after dedup elsewhere.
    std::unordered_map<IslandId, std::vector<Link>> acceptedBySource;
    // Guardrail: source-local collapse must not delete every ingress edge to a nearby target.
    // Retain the best candidate for each target so density pruning stays sparse without turning
    // visually connected islands into directed dead ends.
    std::unordered_map<IslandId, const Link*> preferredIngressByTarget;
    preferredIngressByTarget.reserve(candidates.size());
    const float verticalWindow = effectiveVerticalCollapseWindow(config);
    for (const Link& candidate : candidates) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        preferredIngressByTarget.try_emplace(candidate.toIsland, &candidate);
    }
    std::vector<std::vector<Link>> acceptedOutgoing(islands.size());
    for (const Link& candidate : candidates) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (config.density.globalPruning.enabled) {
            const float globalRadius = config.density.globalPruning.effectiveRadius(
                candidate.horizontalDistance,
                config.gapDiscovery.maxHorizontalGap);
            if (isOccupiedGlobalPoint(candidate.start, globalRadius) ||
                isOccupiedGlobalPoint(candidate.end, globalRadius)) {
                ++stats.candidates.globalPruningRejectCount;
                continue;
            }
        }
        if (config.density.spannerPruning.enabled &&
            hasAcceptableIndirectRoute(
                candidate,
                acceptedOutgoing,
                config.density.spannerPruning.pathRatio,
                config.density.spannerPruning.verticalWeight)) {
            ++stats.candidates.spannerPruningRejectCount;
            continue;
        }

        bool duplicate = false;
        if (config.density.localPruning.enabled) {
            auto& accepted = acceptedBySource[candidate.fromIsland];
            const float radius = pruneRadius(candidate, graph, config);
            const float radiusSquared = radius * radius;
            const bool isPreferredIngress =
                preferredIngressByTarget[candidate.toIsland] == &candidate;
            if (!isPreferredIngress) {
                duplicate = std::any_of(accepted.begin(), accepted.end(), [&](const Link& existing) {
                    // Guardrail: fragmented maps often produce many links that differ only by which
                    // tiny target island owns the far endpoint. Collapse those as one local corridor,
                    // but keep clearly separated vertical layers so stacked traversal routes survive.
                    return withinLocalPruningWindow(candidate, existing, radiusSquared, verticalWindow);
                });
            }
            if (!duplicate) {
                accepted.push_back(candidate);
            } else {
                ++stats.candidates.localPruningRejectCount;
            }
        }
        if (!duplicate) {
            acceptedOutgoing[candidate.fromIsland].push_back(candidate);
            ++stats.candidates.acceptedLinkCount;
            if (config.density.globalPruning.enabled) {
                occupyGlobalPoint(candidate.start);
                occupyGlobalPoint(candidate.end);
            }
        }
    }

    struct EdgeKey {
        IslandId islandA = 0;
        IslandId islandB = 0;
        SpatialCoordinate pointAX = 0;
        SpatialCoordinate pointAY = 0;
        SpatialCoordinate pointAZ = 0;
        SpatialCoordinate pointBX = 0;
        SpatialCoordinate pointBY = 0;
        SpatialCoordinate pointBZ = 0;

        bool operator==(const EdgeKey& other) const {
            return islandA == other.islandA &&
                islandB == other.islandB &&
                pointAX == other.pointAX &&
                pointAY == other.pointAY &&
                pointAZ == other.pointAZ &&
                pointBX == other.pointBX &&
                pointBY == other.pointBY &&
                pointBZ == other.pointBZ;
        }
    };

    struct EdgeKeyHash {
        std::size_t operator()(const EdgeKey& key) const {
            std::size_t hash = 0;
            hashCombine(hash, key.islandA);
            hashCombine(hash, key.islandB);
            hashCombine(hash, key.pointAX);
            hashCombine(hash, key.pointAY);
            hashCombine(hash, key.pointAZ);
            hashCombine(hash, key.pointBX);
            hashCombine(hash, key.pointBY);
            hashCombine(hash, key.pointBZ);
            return hash;
        }
    };

    std::unordered_map<EdgeKey, std::uint32_t, EdgeKeyHash> edgeByKey;
    for (IslandId fromIsland = 0; fromIsland < acceptedOutgoing.size(); ++fromIsland) {
        for (const Link& link : acceptedOutgoing[fromIsland]) {
            const float cellSize = config.density.candidateDeduplication.enabled
                ? config.density.candidateDeduplication.effectiveCellSize(
                    link.horizontalDistance,
                    config.gapDiscovery.maxHorizontalGap)
                : pruneRadius(link, graph, config);
            const bool forward = link.fromIsland <= link.toIsland;
            const EdgeKey key{
                forward ? link.fromIsland : link.toIsland,
                forward ? link.toIsland : link.fromIsland,
                quantize(forward ? link.start.x : link.end.x, cellSize),
                quantize(forward ? link.start.y : link.end.y, verticalWindow),
                quantize(forward ? link.start.z : link.end.z, cellSize),
                quantize(forward ? link.end.x : link.start.x, cellSize),
                quantize(forward ? link.end.y : link.start.y, verticalWindow),
                quantize(forward ? link.end.z : link.start.z, cellSize)};
            const auto existing = edgeByKey.find(key);
            if (existing == edgeByKey.end()) {
                Edge edge;
                edge.islandA = key.islandA;
                edge.islandB = key.islandB;
                edge.pointA = forward ? link.start : link.end;
                edge.pointB = forward ? link.end : link.start;
                edge.horizontalDistance = link.horizontalDistance;
                edge.verticalDeltaAB = edge.pointB.y - edge.pointA.y;
                edge.traversableAB = symmetricCapabilities || forward;
                edge.traversableBA = symmetricCapabilities || !forward;
                edgeByKey.emplace(key, static_cast<std::uint32_t>(edges.size()));
                edges.push_back(edge);
                continue;
            }
            Edge& edge = edges[existing->second];
            if (symmetricCapabilities || forward) {
                edge.traversableAB = true;
            }
            if (symmetricCapabilities || !forward) {
                edge.traversableBA = true;
            }
        }
    }
    IslandGraphAccess::rebuildAdjacency(graph);
    stats.candidates.acceptedLinkCount = edges.size();
    stats.timings.pruningMs = elapsedMilliseconds(pruningStart);
    return BuildStatus::Success;
}

} // namespace detour_island_graph::detail::discovery
