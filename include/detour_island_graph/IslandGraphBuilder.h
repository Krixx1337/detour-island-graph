#pragma once

#include "IslandGraph.h"

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

struct DensityTuning {
    bool enabled = false;
    float distanceScale = 0.0f;
    float maxRadiusScale = 1.0f;
    float globalPruneCellSize = 0.0f;
    bool enableSpannerPruning = false;
    float spannerPathRatio = 1.5f;

    float pruneRadiusScaleFor(float horizontalDistance) const noexcept {
        const float scale = 1.0f + (distanceScale * horizontalDistance);
        return scale < maxRadiusScale ? scale : maxRadiusScale;
    }
};

struct BuildConfig {
    float maxHorizontalGap = 30.0f;
    float maxVerticalGapUp = 30.0f;
    float maxVerticalGapDown = 30.0f;
    float boundaryDeduplicationCellSize = 4.0f;
    float linkDeduplicationCellSize = 8.0f;
    int queryMaxNodes = 4096;
    int maxNearbyPolygons = 2048;
    MassAwareTuning massAware;
    DensityTuning density;
};

struct BuildStats {
    double totalBuildMs = 0.0;
    double floodFillMs = 0.0;
    double massScoringMs = 0.0;
    double boundaryExtractionMs = 0.0;
    double linkDiscoveryMs = 0.0;
    double pruningMs = 0.0;
    std::size_t islandCount = 0;
    std::size_t polygonCount = 0;
    std::size_t rawBoundaryCount = 0;
    std::size_t deduplicatedBoundaryCount = 0;
    std::size_t spatialQueryCount = 0;
    std::size_t nearbyPolygonCount = 0;
    std::size_t projectedCandidateCount = 0;
    std::size_t deduplicatedCandidateCount = 0;
    std::size_t acceptedLinkCount = 0;
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
