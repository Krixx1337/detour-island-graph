#include "IslandGraphDiscoveryInternal.h"

#include "VectorMath.h"

#include <algorithm>
#include <unordered_map>

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
    std::unordered_map<IslandId, std::vector<Link>> acceptedBySource;
    auto& islands = IslandGraphAccess::islands(graph);
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
                islands,
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
            duplicate = std::any_of(accepted.begin(), accepted.end(), [&](const Link& existing) {
                const float startHorizontalDistance =
                    horizontalDistance(candidate.start, existing.start);
                const float endHorizontalDistance =
                    horizontalDistance(candidate.end, existing.end);
                // Guardrail: fragmented maps often produce many links that differ only by which
                // tiny target island owns the far endpoint. Collapse those as one local corridor
                // so source islands keep meaningful exits instead of a full fan-out neighborhood.
                return (startHorizontalDistance * startHorizontalDistance) <= radiusSquared &&
                    (endHorizontalDistance * endHorizontalDistance) <= radiusSquared;
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
                occupyGlobalPoint(candidate.start);
                occupyGlobalPoint(candidate.end);
            }
        }
    }
    stats.timings.pruningMs = elapsedMilliseconds(pruningStart);
    return BuildStatus::Success;
}

} // namespace detour_island_graph::detail::discovery
