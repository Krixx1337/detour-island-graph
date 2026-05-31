#pragma once

#include "IslandGraph.h"

#include <algorithm>
#include <cstddef>
#include <string>

namespace detour_island_graph {

struct MassAwareTuning {
    bool enabled = false;
    float normalizationPercentile = 0.99f;
    float targetPreference = 0.0f;
    float lowMassPruneRadiusScale = 1.0f;
    float highMassPruneRadiusScale = 1.0f;

    float targetPreferenceFor(float massScore) const noexcept {
        return targetPreference * massScore;
    }

    float pruneRadiusScaleFor(float massScore) const noexcept {
        return lowMassPruneRadiusScale +
            ((highMassPruneRadiusScale - lowMassPruneRadiusScale) * massScore);
    }
};

struct CandidateDeduplicationTuning {
    bool enabled = true;
    float cellSize = 0.0f; // 0.0 = auto: maxHorizontalGap * cellSizeRatio
    float cellSizeRatio = 0.25f;

    float effectiveCellSize(float maxHorizontalGap) const noexcept {
        return cellSize > 0.0f ? cellSize : (maxHorizontalGap * cellSizeRatio);
    }
};

struct LocalPruningTuning {
    bool enabled = true;
    float baseRadius = 0.0f; // 0.0 = auto: maxHorizontalGap * baseRadiusRatio
    float baseRadiusRatio = 0.25f;
    bool enableDistanceScaling = false;
    float distanceScale = 0.0f;
    float maxRadiusScale = 1.0f;

    float pruneRadiusScaleFor(float horizontalDistance) const noexcept {
        const float scale = 1.0f + (distanceScale * horizontalDistance);
        return scale < maxRadiusScale ? scale : maxRadiusScale;
    }

    float effectiveBaseRadius(float maxHorizontalGap) const noexcept {
        return baseRadius > 0.0f ? baseRadius : (maxHorizontalGap * baseRadiusRatio);
    }
};

struct GlobalPruningTuning {
    bool enabled = false;
    float cellSize = 0.0f; // 0.0 = auto: maxHorizontalGap * cellSizeRatio
    float cellSizeRatio = 0.5f;

    float effectiveCellSize(float maxHorizontalGap) const noexcept {
        return cellSize > 0.0f ? cellSize : (maxHorizontalGap * cellSizeRatio);
    }
};

struct SpannerPruningTuning {
    bool enabled = false;
    float pathRatio = 1.5f;
    float verticalWeight = 1.0f;
};

struct DensityTuning {
    CandidateDeduplicationTuning candidateDeduplication;
    LocalPruningTuning localPruning;
    GlobalPruningTuning globalPruning;
    SpannerPruningTuning spannerPruning;
};

struct GapDiscoveryTuning {
    float maxHorizontalGap = 30.0f;
    float maxVerticalGapUp = 30.0f;
    float maxVerticalGapDown = 30.0f;
};

struct BoundaryTuning {
    bool deduplicationEnabled = true;
    float deduplicationCellSize = 0.0f; // 0.0 = auto: maxHorizontalGap * deduplicationCellSizeRatio
    float deduplicationCellSizeRatio = 0.125f;
    bool representativeReductionEnabled = false;
    float representativeCellSize = 0.0f; // 0.0 = auto: maxHorizontalGap * representativeCellSizeRatio
    float representativeCellSizeRatio = 0.25f;

    float effectiveDeduplicationCellSize(float maxHorizontalGap) const noexcept {
        return deduplicationCellSize > 0.0f
            ? deduplicationCellSize
            : (maxHorizontalGap * deduplicationCellSizeRatio);
    }

    float effectiveRepresentativeCellSize(float maxHorizontalGap) const noexcept {
        return representativeCellSize > 0.0f
            ? representativeCellSize
            : (maxHorizontalGap * representativeCellSizeRatio);
    }
};

struct QueryTuning {
    int maxNodes = 4096;
    int maxNearbyPolygons = 2048;
};

struct BuildConfig {
    GapDiscoveryTuning gapDiscovery;
    BoundaryTuning boundaries;
    QueryTuning query;
    MassAwareTuning massAware;
    DensityTuning density;
};

struct TimingStats {
    double totalMs = 0.0;
    double floodFillMs = 0.0;
    double massScoringMs = 0.0;
    double boundaryExtractionMs = 0.0;
    double linkDiscoveryMs = 0.0;
    double pruningMs = 0.0;
};

struct BoundaryStats {
    std::size_t rawCount = 0;
    std::size_t deduplicatedCount = 0;
    std::size_t representativeCount = 0;
    std::size_t representativeTrimmedCount = 0;
};

struct QueryStats {
    std::size_t count = 0;
    std::size_t capacityHitCount = 0;
    std::size_t nearbyPolygonCount = 0;
};

struct CandidateStats {
    std::size_t projectedCount = 0;
    std::size_t deduplicatedCount = 0;
    std::size_t acceptedLinkCount = 0;
    std::size_t globalPruningRejectCount = 0;
    std::size_t spannerPruningRejectCount = 0;
    std::size_t localPruningRejectCount = 0;
};

struct BuildStats {
    TimingStats timings;
    BoundaryStats boundaries;
    QueryStats queries;
    CandidateStats candidates;
    std::size_t islandCount = 0;
    std::size_t polygonCount = 0;
    std::size_t islandsWithOutgoingLinks = 0;
    std::size_t islandsWithIncomingLinks = 0;
    std::size_t isolatedIslandCount = 0;
    std::size_t connectedComponentCount = 0;
    std::size_t largestConnectedComponentIslandCount = 0;
    std::size_t maxOutgoingLinksOnIsland = 0;
    std::size_t p95OutgoingLinksOnIsland = 0;
    std::size_t maxIncomingLinksOnIsland = 0;
    std::size_t p95IncomingLinksOnIsland = 0;
    double averageLinkLength = 0.0;
};

enum class BuildStatus {
    Success,
    InvalidConfiguration,
    QueryInitializationFailed,
    QueryFailed
};

struct BuildResult {
    IslandGraph graph;
    BuildStats stats;
    BuildStatus status = BuildStatus::Success;
    std::string message;

    explicit operator bool() const noexcept {
        return status == BuildStatus::Success;
    }
};

class IslandGraphBuilder {
public:
    BuildResult build(const dtNavMesh& navMesh, const BuildConfig& config = {}) const;
};

} // namespace detour_island_graph
