#include "IslandGraphDiscoveryInternal.h"

#include "VectorMath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

std::int64_t quantizedMillimeters(float value) {
    const double scaled = static_cast<double>(value) * 1000.0;
    if (!std::isfinite(scaled)) {
        return 0;
    }
    if (scaled <= static_cast<double>((std::numeric_limits<std::int64_t>::min)())) {
        return (std::numeric_limits<std::int64_t>::min)();
    }
    if (scaled >= static_cast<double>((std::numeric_limits<std::int64_t>::max)())) {
        return (std::numeric_limits<std::int64_t>::max)();
    }
    return static_cast<std::int64_t>(std::llround(scaled));
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
    return config.density.localPruning.effectiveRadius(maxTraversalExtent(config)) * scale;
}

float globalPruneRadius(
    const Link& link,
    IslandId endpointIsland,
    const IslandGraph& graph,
    const BuildConfig& config) {
    float radius = config.density.globalPruning.effectiveRadius(maxTraversalExtent(config));
    if (config.massAware.enabled && endpointIsland < graph.islands().size()) {
        radius *= config.density.globalPruning.massRadiusScaleFor(
            graph.islands()[endpointIsland].massScore);
    }
    return radius;
}

bool withinLocalPruningWindow(
    const Link& candidate,
    const Link& existing,
    float radiusSquared) {
    return distanceSquared(candidate.start, existing.start) <= radiusSquared &&
        distanceSquared(candidate.end, existing.end) <= radiusSquared;
}

Link reverseLink(const Link& link) {
    Link reversed = link;
    std::swap(reversed.fromIsland, reversed.toIsland);
    std::swap(reversed.start, reversed.end);
    reversed.verticalDistance = -reversed.verticalDistance;
    return reversed;
}

bool hasAcceptableIndirectRoute(
    const Link& candidate,
    const std::vector<std::vector<Link>>& acceptedOutgoing,
    float pathRatio) {
    const auto& outgoing = acceptedOutgoing[candidate.fromIsland];
    const auto effort = [](const Link& link) {
        return distance(link.start, link.end);
    };
    const float candidateDist = effort(candidate);

    for (const Link& firstHop : outgoing) {
        if (firstHop.toIsland == candidate.toIsland) {
            continue;
        }
        const auto& secondOutgoing = acceptedOutgoing[firstHop.toIsland];
        for (const Link& secondHop : secondOutgoing) {
            if (secondHop.toIsland == candidate.toIsland) {
                const float indirectDist =
                    distance(candidate.start, firstHop.start) + effort(firstHop) +
                    distance(firstHop.end, secondHop.start) + effort(secondHop) +
                    distance(secondHop.end, candidate.end);
                if (indirectDist <= candidateDist * pathRatio) {
                    return true;
                }
            }
        }
    }
    return false;
}

std::vector<std::uint32_t> makeDistinctTargetReserveBudgets(
    const IslandGraph& graph,
    const BuildConfig& config) {
    if (!config.density.spannerPruning.enabled || !config.density.distinctTargetReserve.enabled) {
        return {};
    }
    std::vector<std::uint32_t> budgets(graph.islands().size(), 0);
    for (const Island& island : graph.islands()) {
        if (island.id >= budgets.size()) {
            continue;
        }
        budgets[island.id] = config.density.distinctTargetReserve.targetCountFor(
            island.massScore,
            config.massAware.enabled);
    }
    return budgets;
}

bool needsDistinctTargetReserve(
    IslandId sourceIsland,
    IslandId targetIsland,
    const std::vector<std::uint32_t>& budgets,
    const std::vector<std::unordered_set<IslandId>>& reservedTargets) {
    return sourceIsland < budgets.size() &&
        sourceIsland < reservedTargets.size() &&
        reservedTargets[sourceIsland].size() < budgets[sourceIsland] &&
        reservedTargets[sourceIsland].find(targetIsland) == reservedTargets[sourceIsland].end();
}

void reserveDistinctTarget(
    IslandId sourceIsland,
    IslandId targetIsland,
    const std::vector<std::uint32_t>& budgets,
    std::vector<std::unordered_set<IslandId>>& reservedTargets) {
    if (needsDistinctTargetReserve(sourceIsland, targetIsland, budgets, reservedTargets)) {
        reservedTargets[sourceIsland].insert(targetIsland);
    }
}

} // namespace

bool isBetterLink(const Link& lhs, const Link& rhs, const IslandGraph& graph, const BuildConfig& config) {
    if (config.massAware.enabled) {
        const float lhsImportance = linkImportance(lhs, graph);
        const float rhsImportance = linkImportance(rhs, graph);
        if (lhsImportance != rhsImportance) {
            return lhsImportance > rhsImportance;
        }
    }
    const float lhsRank = rankScore(lhs, graph, config);
    const float rhsRank = rankScore(rhs, graph, config);
    const std::int64_t lhsRankMm = quantizedMillimeters(lhsRank);
    const std::int64_t rhsRankMm = quantizedMillimeters(rhsRank);
    if (lhsRankMm != rhsRankMm) return lhsRankMm < rhsRankMm;
    const std::int64_t lhsDistanceMm = quantizedMillimeters(linkDistance(lhs));
    const std::int64_t rhsDistanceMm = quantizedMillimeters(linkDistance(rhs));
    if (lhsDistanceMm != rhsDistanceMm) return lhsDistanceMm < rhsDistanceMm;
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
    std::vector<bool> outboundIslands(islands.size(), true);
    if (config.outboundIslandFilter) {
        for (const Island& island : islands) {
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            outboundIslands[island.id] = !island.suppressed && config.outboundIslandFilter(island, graph);
        }
    } else {
        for (const Island& island : islands) {
            if (island.id < outboundIslands.size()) {
                outboundIslands[island.id] = !island.suppressed;
            }
        }
    }
    const auto reverseAllowed = [&](const Link& link) {
        const double reverseRise = static_cast<double>(link.start.y) - link.end.y;
        return outboundIslands[link.toIsland] &&
            reverseRise <= config.gapDiscovery.maxVerticalGapUp &&
            reverseRise >= -config.gapDiscovery.maxVerticalGapDown;
    };
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
            : config.density.globalPruning.effectiveRadius(maxTraversalExtent(config)))
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
    std::vector<std::vector<Link>> acceptedBySource(
        config.density.localPruning.enabled ? islands.size() : 0);
    std::vector<Link> locallyAccepted;
    locallyAccepted.reserve(candidates.size());
    std::vector<std::vector<Link>> acceptedOutgoing(islands.size());
    std::vector<std::vector<Link>> spannerOutgoing(
        config.density.spannerPruning.enabled ? islands.size() : 0);
    const std::vector<std::uint32_t> distinctTargetReserveBudgets =
        makeDistinctTargetReserveBudgets(graph, config);
    std::vector<std::unordered_set<IslandId>> spannerReservedTargets(distinctTargetReserveBudgets.size());
    for (const Link& candidate : candidates) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (candidate.fromIsland >= islands.size() ||
            candidate.toIsland >= islands.size() ||
            candidate.fromIsland == candidate.toIsland ||
            !outboundIslands[candidate.fromIsland] ||
            islands[candidate.toIsland].suppressed ||
            !withinTraversalLimits(candidate.start, candidate.end, config)) {
            continue;
        }
        bool duplicate = false;
        if (config.density.localPruning.enabled) {
            auto& accepted = acceptedBySource[candidate.fromIsland];
            const float radius = pruneRadius(candidate, graph, config);
            const float radiusSquared = radius * radius;
            duplicate = std::any_of(accepted.begin(), accepted.end(), [&](const Link& existing) {
                return candidate.toIsland == existing.toIsland &&
                    (!reverseAllowed(candidate) || reverseAllowed(existing)) &&
                    withinLocalPruningWindow(candidate, existing, radiusSquared);
            });
            if (!duplicate) {
                accepted.push_back(candidate);
                if (reverseAllowed(candidate)) {
                    const Link reverse = reverseLink(candidate);
                    acceptedBySource[candidate.toIsland].push_back(reverse);
                }
            } else {
                ++stats.candidates.localPruningRejectCount;
            }
        }
        if (!duplicate) {
            locallyAccepted.push_back(candidate);
        }
    }

    for (const Link& candidate : locallyAccepted) {
        if (cancellationRequested(options)) {
            return BuildStatus::Cancelled;
        }
        if (config.density.globalPruning.enabled) {
            const float sourceRadius = globalPruneRadius(candidate, candidate.fromIsland, graph, config);
            const float targetRadius = globalPruneRadius(candidate, candidate.toIsland, graph, config);
            if (isOccupiedGlobalPoint(candidate.start, sourceRadius) ||
                isOccupiedGlobalPoint(candidate.end, targetRadius)) {
                ++stats.candidates.globalPruningRejectCount;
                continue;
            }
        }
        const bool hasReplacement = config.density.spannerPruning.enabled &&
            hasAcceptableIndirectRoute(
                candidate,
                spannerOutgoing,
                config.density.spannerPruning.pathRatio) &&
            (!reverseAllowed(candidate) || hasAcceptableIndirectRoute(
                reverseLink(candidate), spannerOutgoing, config.density.spannerPruning.pathRatio));
        if (hasReplacement) {
            const bool reserved = needsDistinctTargetReserve(
                candidate.fromIsland, candidate.toIsland,
                distinctTargetReserveBudgets, spannerReservedTargets) ||
                (reverseAllowed(candidate) && needsDistinctTargetReserve(
                    candidate.toIsland, candidate.fromIsland,
                    distinctTargetReserveBudgets, spannerReservedTargets));
            if (reserved) {
                ++stats.candidates.distinctTargetReserveCount;
            } else {
                ++stats.candidates.spannerPruningRejectCount;
                continue;
            }
        }

        acceptedOutgoing[candidate.fromIsland].push_back(candidate);
        if (config.density.spannerPruning.enabled) {
            spannerOutgoing[candidate.fromIsland].push_back(candidate);
            reserveDistinctTarget(
                candidate.fromIsland,
                candidate.toIsland,
                distinctTargetReserveBudgets,
                spannerReservedTargets);
            if (reverseAllowed(candidate)) {
                const Link reverse = reverseLink(candidate);
                spannerOutgoing[candidate.toIsland].push_back(reverse);
                reserveDistinctTarget(
                    reverse.fromIsland,
                    reverse.toIsland,
                    distinctTargetReserveBudgets,
                    spannerReservedTargets);
            }
        }
        ++stats.candidates.acceptedLinkCount;
        if (config.density.globalPruning.enabled) {
            occupyGlobalPoint(candidate.start);
            occupyGlobalPoint(candidate.end);
        }
    }

    stats.candidates.acceptedLinkCount = 0;
    for (const auto& outgoing : acceptedOutgoing) {
        stats.candidates.acceptedLinkCount += outgoing.size();
    }

    struct EdgeKey {
        IslandId islandA = 0;
        IslandId islandB = 0;
        float pointAX = 0;
        float pointAY = 0;
        float pointAZ = 0;
        float pointBX = 0;
        float pointBY = 0;
        float pointBZ = 0;

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
            if (cancellationRequested(options)) {
                return BuildStatus::Cancelled;
            }
            const bool forward = link.fromIsland <= link.toIsland;
            const EdgeKey key{
                forward ? link.fromIsland : link.toIsland,
                forward ? link.toIsland : link.fromIsland,
                forward ? link.start.x : link.end.x,
                forward ? link.start.y : link.end.y,
                forward ? link.start.z : link.end.z,
                forward ? link.end.x : link.start.x,
                forward ? link.end.y : link.start.y,
                forward ? link.end.z : link.start.z};
            const auto existing = edgeByKey.find(key);
            if (existing == edgeByKey.end()) {
                Edge edge;
                edge.islandA = key.islandA;
                edge.islandB = key.islandB;
                edge.pointA = forward ? link.start : link.end;
                edge.pointB = forward ? link.end : link.start;
                edge.horizontalDistance = horizontalDistance(edge.pointA, edge.pointB);
                edge.verticalDeltaAB = edge.pointB.y - edge.pointA.y;
                edge.traversableAB = outboundIslands[edge.islandA] &&
                    withinTraversalLimits(edge.pointA, edge.pointB, config);
                edge.traversableBA = outboundIslands[edge.islandB] &&
                    withinTraversalLimits(edge.pointB, edge.pointA, config);
                edgeByKey.emplace(key, static_cast<std::uint32_t>(edges.size()));
                edges.push_back(edge);
                continue;
            }
        }
    }
    IslandGraphAccess::rebuildAdjacency(graph);
    stats.candidates.acceptedLinkCount = edges.size();
    // Guardrail: structural validation reads the stored graph against the effective build
    // capability, so it must run after final edge assembly and adjacency rebuild.
    const BuildStatus validityStatus = calculateDirectionValidity(graph, config, options, stats);
    if (validityStatus != BuildStatus::Success) {
        return validityStatus;
    }
    stats.timings.pruningMs = elapsedMilliseconds(pruningStart);
    return BuildStatus::Success;
}

} // namespace detour_island_graph::detail::discovery
