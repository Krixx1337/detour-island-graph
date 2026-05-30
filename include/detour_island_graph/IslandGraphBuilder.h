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
    float cellSizeNear = 0.0f; // 0.0 = auto: maxHorizontalGap * 0.25f
    float cellSizeFar = 0.0f;  // 0.0 = auto: maxHorizontalGap * 1.0f

    float cellSizeFor(float horizontalDistance, float maxHorizontalGap) const noexcept {
        const float nearVal = cellSizeNear > 0.0f ? cellSizeNear : (maxHorizontalGap * 0.25f);
        const float farVal = cellSizeFar > 0.0f ? cellSizeFar : maxHorizontalGap;
        const float alpha = maxHorizontalGap > 0.0f
            ? std::clamp(horizontalDistance / maxHorizontalGap, 0.0f, 1.0f)
            : 0.0f;
        return nearVal + ((farVal - nearVal) * alpha);
    }
};

struct LocalPruningTuning {
    bool enabled = true;
    float baseRadius = 0.0f; // 0.0 = auto: maxHorizontalGap * 0.25f
    bool enableDistanceScaling = false;
    float distanceScale = 0.0f;
    float maxRadiusScale = 1.0f;

    float pruneRadiusScaleFor(float horizontalDistance) const noexcept {
        const float scale = 1.0f + (distanceScale * horizontalDistance);
        return scale < maxRadiusScale ? scale : maxRadiusScale;
    }

    float effectiveBaseRadius(float maxHorizontalGap) const noexcept {
        return baseRadius > 0.0f ? baseRadius : (maxHorizontalGap * 0.25f);
    }
};

struct GlobalPruningTuning {
    bool enabled = false;
    // The global occupancy cell size is proportional to the candidate link's
    // horizontal distance.  No hardcoded distances.  No thresholds.
    //   cellSize = horizontalDistance * relativeCellSize
    // Example: 1.0 => a 30m leap uses a 30m cell; a 2m hop uses a 2m cell.
    float relativeCellSize = 1.0f;

    float cellSizeFor(float horizontalDistance) const noexcept {
        return horizontalDistance * std::max(relativeCellSize, 0.0f);
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
    float deduplicationCellSize = 0.0f; // 0.0 = auto: maxHorizontalGap * 0.125f

    float effectiveDeduplicationCellSize(float maxHorizontalGap) const noexcept {
        return deduplicationCellSize > 0.0f ? deduplicationCellSize : (maxHorizontalGap * 0.125f);
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
};

struct BuildStats {
    TimingStats timings;
    BoundaryStats boundaries;
    QueryStats queries;
    CandidateStats candidates;
    std::size_t islandCount = 0;
    std::size_t polygonCount = 0;
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
