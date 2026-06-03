#pragma once

#include "IslandGraph.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <functional>
#include <string>

namespace detour_island_graph {

struct MassAwareTuning {
    bool enabled = false;
    float normalizationPercentile = 0.99f;
    // Zero keeps the autoscaled preference derived from maxHorizontalGap * targetPreferenceRatio.
    // Set an explicit meter value only when a deployment has benchmarked map-specific tuning.
    float targetPreference = 0.0f;
    float targetPreferenceRatio = 0.0f;
    float lowMassPruneRadiusScale = 1.0f;
    float highMassPruneRadiusScale = 1.0f;

    float effectiveTargetPreference(float maxHorizontalGap) const noexcept {
        return targetPreference > 0.0f
            ? targetPreference
            : (maxHorizontalGap * targetPreferenceRatio);
    }

    float targetPreferenceFor(float massScore, float maxHorizontalGap) const noexcept {
        return effectiveTargetPreference(maxHorizontalGap) * massScore;
    }

    float pruneRadiusScaleFor(float massScore) const noexcept {
        return lowMassPruneRadiusScale +
            ((highMassPruneRadiusScale - lowMassPruneRadiusScale) * massScore);
    }
};

struct CandidateDeduplicationTuning {
    bool enabled = true;
    // Zero keeps the autoscaled voxel size derived from maxTraversalExtent * cellSizeRatio.
    // Set an explicit meter value only for benchmarked map-specific tuning.
    float cellSize = 0.0f;
    float cellSizeRatio = 0.25f;

    float effectiveCellSize(float maxTraversalExtent) const noexcept {
        if (cellSize > 0.0f) {
            return cellSize;
        }
        return maxTraversalExtent * cellSizeRatio;
    }
};

struct PairScanSuppressionTuning {
    bool enabled = false;
    // Zero keeps the autoscaled voxel size derived from maxTraversalExtent * cellSizeRatio.
    // Set an explicit meter value only for benchmarked map-specific tuning.
    float cellSize = 0.0f;
    float cellSizeRatio = 0.25f;

    float effectiveCellSize(float maxTraversalExtent) const noexcept {
        return cellSize > 0.0f ? cellSize : (maxTraversalExtent * cellSizeRatio);
    }
};

struct ShortGapRecoveryTuning {
    bool enabled = false;
    // Zero keeps the autoscaled recovery gap derived from maxHorizontalGap * maxHorizontalGapRatio.
    // The effective value is still clamped to the configured traversal capability.
    float maxHorizontalGap = 0.0f;
    float maxHorizontalGapRatio = 0.2f;

    float effectiveMaxHorizontalGap(float configuredMaxHorizontalGap) const noexcept {
        const float requestedGap = maxHorizontalGap > 0.0f
            ? maxHorizontalGap
            : (configuredMaxHorizontalGap * maxHorizontalGapRatio);
        return (std::min)(requestedGap, configuredMaxHorizontalGap);
    }
};

struct LocalPruningTuning {
    bool enabled = true;
    // Zero keeps the autoscaled pruning radius derived from maxTraversalExtent * radiusRatio.
    // Set an explicit meter value only for benchmarked map-specific tuning.
    float radius = 0.0f;
    float radiusRatio = 0.25f;

    float effectiveRadius(float maxTraversalExtent) const noexcept {
        return radius > 0.0f ? radius : (maxTraversalExtent * radiusRatio);
    }
};

struct GlobalPruningTuning {
    bool enabled = false;
    // Zero keeps the autoscaled pruning radius derived from maxTraversalExtent * radiusRatio.
    // Global pruning is intentionally optional because endpoint proximity is a coarse heuristic.
    float radius = 0.0f;
    float radiusRatio = 0.5f;
    float lowMassRadiusScale = 1.0f;
    float highMassRadiusScale = 1.0f;

    float effectiveRadius(float maxTraversalExtent) const noexcept {
        if (radius > 0.0f) {
            return radius;
        }
        return maxTraversalExtent * radiusRatio;
    }

    float massRadiusScaleFor(float massScore) const noexcept {
        return lowMassRadiusScale +
            ((highMassRadiusScale - lowMassRadiusScale) * massScore);
    }
};

struct SpannerPruningTuning {
    bool enabled = false;
    float pathRatio = 1.5f;
};

struct DensityTuning {
    float verticalLayerCollapseRatio = 0.25f;
    PairScanSuppressionTuning pairScanSuppression;
    ShortGapRecoveryTuning shortGapRecovery;
    CandidateDeduplicationTuning candidateDeduplication;
    LocalPruningTuning localPruning;
    GlobalPruningTuning globalPruning;
    SpannerPruningTuning spannerPruning;
};

struct GapDiscoveryTuning {
    float maxHorizontalGap;
    float maxVerticalGapUp;
    float maxVerticalGapDown;
};

struct BoundaryRepresentativeCandidate {
    IslandId island = 0;
    dtPolyRef polygon = 0;
    Vec3 start;
    Vec3 end;
    Vec3 midpoint;
};

using BoundaryRepresentativeRanker =
    std::function<float(const BoundaryRepresentativeCandidate&, const IslandGraph&)>;

struct BoundaryTuning {
    bool deduplicationEnabled = true;
    // Zero keeps the autoscaled voxel size derived from maxHorizontalGap * deduplicationCellSizeRatio.
    float deduplicationCellSize = 0.0f;
    float deduplicationCellSizeRatio = 0.125f;
    bool representativeReductionEnabled = false;
    // Zero keeps the autoscaled voxel size derived from maxHorizontalGap * representativeCellSizeRatio.
    float representativeCellSize = 0.0f;
    float representativeCellSizeRatio = 0.25f;
    int representativeDirectionBuckets = 8;
    BoundaryRepresentativeRanker representativeRanker;

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

    int representativeDirectionBucket(const Vec3& direction) const noexcept {
        constexpr float tau = 6.28318530718f;
        const int bucketCount = (std::max)(representativeDirectionBuckets, 1);
        const float angle = std::atan2(direction.z, direction.x);
        const float normalizedAngle = angle < 0.0f ? angle + tau : angle;
        return static_cast<int>(
            std::floor(normalizedAngle * static_cast<float>(bucketCount) / tau)) %
            bucketCount;
    }
};

struct QueryTuning {
    int maxNodes = 8192;
    unsigned short includeFlags = 0xffff;
    unsigned short excludeFlags = 0;
};

using PolygonFilter = std::function<bool(dtPolyRef, const dtMeshTile&, const dtPoly&)>;
using OutboundIslandFilter = std::function<bool(const Island&, const IslandGraph&)>;
using LinkRanker = std::function<float(const Link&, const IslandGraph&)>;

enum class BuildProfile {
    Conservative,
    Sparse,
    Unpruned
};

struct BuildConfig {
    BuildConfig() = delete;

    explicit BuildConfig(
        float maxHorizontalGap,
        float maxVerticalGapUp,
        float maxVerticalGapDown)
        : gapDiscovery{
            maxHorizontalGap,
            maxVerticalGapUp,
            maxVerticalGapDown} {}

    [[nodiscard]] static BuildConfig forProfile(
        BuildProfile profile,
        float maxHorizontalGap,
        float maxVerticalGapUp,
        float maxVerticalGapDown);

    GapDiscoveryTuning gapDiscovery;
    BoundaryTuning boundaries;
    QueryTuning query;
    MassAwareTuning massAware;
    DensityTuning density;
    PolygonFilter polygonFilter;
    OutboundIslandFilter outboundIslandFilter;
    LinkRanker linkRanker;
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
    std::size_t outboundFilteredCount = 0;
    std::size_t representativeCount = 0;
    std::size_t representativeTrimmedCount = 0;
};

struct QueryStats {
    std::size_t count = 0;
    std::size_t nearbyPolygonCount = 0;
};

struct CandidateStats {
    std::size_t pairScanCandidateCount = 0;
    std::size_t pairScanSuppressedCount = 0;
    std::size_t shortGapRecoveryQueryCount = 0;
    std::size_t shortGapRecoveredCount = 0;
    std::size_t reverseLinksSynthesizedCount = 0;
    std::size_t reverseLinksRejectedCount = 0;
    std::size_t closestPointQueryCount = 0;
    std::size_t closestPointFailureCount = 0;
    std::size_t projectedCount = 0;
    std::size_t deduplicatedCount = 0;
    std::size_t acceptedLinkCount = 0;
    std::size_t globalPruningRejectCount = 0;
    std::size_t spannerPruningRejectCount = 0;
    std::size_t localPruningRejectCount = 0;
};

struct MassBucketStats {
    std::size_t islandCount = 0;
    std::size_t outgoingLinkCount = 0;
    std::size_t incomingLinkCount = 0;
    std::size_t isolatedIslandCount = 0;
    std::size_t maxOutgoingLinksOnIsland = 0;
    std::size_t p95OutgoingLinksOnIsland = 0;
    std::size_t maxIncomingLinksOnIsland = 0;
    std::size_t p95IncomingLinksOnIsland = 0;
    double totalMass = 0.0;
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
    std::size_t isolatedIslandPolygonCount = 0;
    std::size_t largestConnectedComponentPolygonCount = 0;
    double totalIslandMass = 0.0;
    double isolatedIslandMass = 0.0;
    double largestConnectedComponentMass = 0.0;
    std::size_t maxOutgoingLinksOnIsland = 0;
    std::size_t p95OutgoingLinksOnIsland = 0;
    std::size_t maxIncomingLinksOnIsland = 0;
    std::size_t p95IncomingLinksOnIsland = 0;
    std::array<MassBucketStats, 3> massBuckets{};
    double averageLinkLength = 0.0;
};

enum class BuildStatus {
    Success,
    Cancelled,
    InvalidConfiguration,
    InvalidNavMesh,
    QueryInitializationFailed,
    QueryFailed
};

struct BuildOptions {
    std::function<bool()> shouldCancel;
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
    [[nodiscard]] BuildResult build(
        const dtNavMesh& navMesh,
        const BuildConfig& config,
        const BuildOptions& options = {}) const;
};

} // namespace detour_island_graph
