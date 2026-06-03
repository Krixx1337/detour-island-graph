#pragma once

#include "IslandGraph.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <functional>
#include <string>

namespace detour_island_graph {

struct MassAwareTuning {
    bool enabled = false;
    float normalizationPercentile = 0.99f;
    float targetPreference = 0.0f; // 0.0 = auto: maxHorizontalGap * targetPreferenceRatio
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
    float cellSize = 0.0f; // 0.0 = auto: interpolate ratios across the horizontal gap range
    float nearCellSizeRatio = 0.25f;
    float farCellSizeRatio = 0.25f;

    float effectiveCellSize(float horizontalDistance, float maxHorizontalGap) const noexcept {
        if (cellSize > 0.0f) {
            return cellSize;
        }
        const float alpha = std::clamp(horizontalDistance / maxHorizontalGap, 0.0f, 1.0f);
        const float ratio = nearCellSizeRatio + (alpha * (farCellSizeRatio - nearCellSizeRatio));
        return maxHorizontalGap * ratio;
    }
};

struct PairScanSuppressionTuning {
    bool enabled = false;
    float cellSize = 0.0f; // 0.0 = auto: maxHorizontalGap * cellSizeRatio
    float cellSizeRatio = 0.25f;

    float effectiveCellSize(float maxHorizontalGap) const noexcept {
        return cellSize > 0.0f ? cellSize : (maxHorizontalGap * cellSizeRatio);
    }
};

struct ShortGapRecoveryTuning {
    bool enabled = false;
    float maxHorizontalGap = 0.0f; // 0.0 = auto: gapDiscovery.maxHorizontalGap * maxHorizontalGapRatio
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
    float baseRadius = 0.0f; // 0.0 = auto: maxHorizontalGap * baseRadiusRatio
    float baseRadiusRatio = 0.25f;
    bool enableDistanceScaling = false;
    float distanceScale = 0.0f; // 0.0 = auto: distanceScaleRatio / maxHorizontalGap
    float distanceScaleRatio = 0.0f;
    float maxRadiusScale = 1.0f;

    float effectiveDistanceScale(float maxHorizontalGap) const noexcept {
        return distanceScale > 0.0f
            ? distanceScale
            : (distanceScaleRatio / maxHorizontalGap);
    }

    float pruneRadiusScaleFor(float horizontalDistance, float maxHorizontalGap) const noexcept {
        const float scale = 1.0f + (effectiveDistanceScale(maxHorizontalGap) * horizontalDistance);
        return scale < maxRadiusScale ? scale : maxRadiusScale;
    }

    float effectiveBaseRadius(float maxHorizontalGap) const noexcept {
        return baseRadius > 0.0f ? baseRadius : (maxHorizontalGap * baseRadiusRatio);
    }
};

struct GlobalPruningTuning {
    bool enabled = false;
    float radius = 0.0f; // 0.0 = auto: interpolate ratios across the horizontal gap range
    float nearRadiusRatio = 0.5f;
    float farRadiusRatio = 0.5f;
    float lowMassRadiusScale = 1.0f;
    float highMassRadiusScale = 1.0f;

    float effectiveRadius(float horizontalDistance, float maxHorizontalGap) const noexcept {
        if (radius > 0.0f) {
            return radius;
        }
        const float alpha = std::clamp(horizontalDistance / maxHorizontalGap, 0.0f, 1.0f);
        const float ratio = nearRadiusRatio + (alpha * (farRadiusRatio - nearRadiusRatio));
        return maxHorizontalGap * ratio;
    }

    float massRadiusScaleFor(float massScore) const noexcept {
        return lowMassRadiusScale +
            ((highMassRadiusScale - lowMassRadiusScale) * massScore);
    }
};

struct SpannerPruningTuning {
    bool enabled = false;
    float pathRatio = 1.5f;
    float verticalWeight = 1.0f;
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
    float deduplicationCellSize = 0.0f; // 0.0 = auto: maxHorizontalGap * deduplicationCellSizeRatio
    float deduplicationCellSizeRatio = 0.125f;
    bool representativeReductionEnabled = false;
    float representativeCellSize = 0.0f; // 0.0 = auto: maxHorizontalGap * representativeCellSizeRatio
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
