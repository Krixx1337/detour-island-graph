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

struct BuildConfig {
    float maxHorizontalGap = 30.0f;
    float maxVerticalGapUp = 30.0f;
    float maxVerticalGapDown = 30.0f;
    float boundaryDeduplicationCellSize = 4.0f;
    float linkDeduplicationCellSize = 8.0f;
    int queryMaxNodes = 4096;
    int maxNearbyPolygons = 2048;
    MassAwareTuning massAware;
};

enum class BuildStatus {
    Success,
    InvalidConfiguration,
    QueryInitializationFailed,
    QueryFailed
};

struct BuildResult {
    IslandGraph graph;
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
