#pragma once

#include "IslandGraph.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace detour_island_graph {

struct MassAwareTuning {
    bool enabled = false;
    bool suppressSmallIslands = false;
    float normalizationPercentile = 0.99f;
    float suppressedIslandPercent = 0.0f;
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
    // Applied only with equal climb/drop limits; asymmetric scans retain direction classes.
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
    // Collapses nearby corridors only within the same island pair, preserving valid directions.
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
    // Uses crossing lengths plus Euclidean transfers between corridor endpoints.
    float pathRatio = 1.5f;
};

struct DistinctTargetReserveTuning {
    bool enabled = false;
    std::uint32_t minTargetsPerIsland = 0;
    std::uint32_t maxTargetsPerIsland = 0;
    float massPower = 1.0f;

    std::uint32_t targetCountFor(float massScore, bool massAware) const noexcept {
        if (!enabled || maxTargetsPerIsland == 0) {
            return 0;
        }
        if (!massAware) {
            return maxTargetsPerIsland;
        }
        const float mass = std::clamp(massScore, 0.0f, 1.0f);
        const float weightedMass = std::pow(mass, massPower);
        float targetCount = static_cast<float>(minTargetsPerIsland);
        if (minTargetsPerIsland < maxTargetsPerIsland) {
            targetCount +=
                static_cast<float>(maxTargetsPerIsland - minTargetsPerIsland) *
                weightedMass;
        } else {
            targetCount = static_cast<float>(maxTargetsPerIsland);
        }
        return (std::min)(
            static_cast<std::uint32_t>((std::max)(0.0f, std::floor(targetCount))),
            maxTargetsPerIsland);
    }
};

struct DensityTuning {
    float verticalLayerCollapseRatio = 0.25f;
    PairScanSuppressionTuning pairScanSuppression;
    ShortGapRecoveryTuning shortGapRecovery;
    CandidateDeduplicationTuning candidateDeduplication;
    LocalPruningTuning localPruning;
    GlobalPruningTuning globalPruning;
    SpannerPruningTuning spannerPruning;
    DistinctTargetReserveTuning distinctTargetReserve;
};

struct GapDiscoveryTuning {
    // Independent horizontal and signed vertical limits, in navmesh coordinate units.
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
    // Zero max keeps all reduced representatives. When mass-aware tuning is enabled, min/max
    // define a smooth per-island representative budget from low mass to high mass.
    std::uint32_t minRepresentativesPerIsland = 0;
    std::uint32_t maxRepresentativesPerIsland = 0;
    float representativeMassPower = 1.0f;
    // Zero disables complexity scaling. When enabled, budget grows with sqrt(reduced reps on island).
    float representativeBudgetScale = 0.0f;
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

    std::uint32_t representativeBudgetFor(
        float massScore,
        bool massAware,
        std::size_t availableRepresentatives = 0) const noexcept {
        if (maxRepresentativesPerIsland == 0 && representativeBudgetScale <= 0.0f) {
            return 0;
        }
        if (!massAware) {
            return maxRepresentativesPerIsland;
        }
        const float mass = std::clamp(massScore, 0.0f, 1.0f);
        const float weightedMass = std::pow(mass, representativeMassPower);
        float budget = static_cast<float>(minRepresentativesPerIsland);
        if (representativeBudgetScale > 0.0f) {
            budget += representativeBudgetScale *
                std::sqrt(static_cast<float>(availableRepresentatives)) *
                weightedMass;
        } else if (minRepresentativesPerIsland < maxRepresentativesPerIsland) {
            budget +=
                static_cast<float>(maxRepresentativesPerIsland - minRepresentativesPerIsland) *
                weightedMass;
        } else {
            budget = static_cast<float>(maxRepresentativesPerIsland);
        }
        const std::uint32_t roundedBudget = static_cast<std::uint32_t>((std::max)(0.0f, std::floor(budget)));
        return maxRepresentativesPerIsland > 0
            ? (std::min)(roundedBudget, maxRepresentativesPerIsland)
            : roundedBudget;
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
    // Scanned boundary representatives per island (indexed by island id). Sized during
    // representative selection; empty when that stage never ran (e.g. deserialized stats).
    std::vector<std::size_t> representativeCountByIsland;
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
    std::size_t distinctTargetReserveCount = 0;
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

// Dominant-island row. The largest island is selected by polygon count (ties break toward
// the smaller id); polygon share is a cheap tessellation-sensitive approximation, not
// walkable area. Distinct-neighbor counts separate useful distributed exits from redundant
// same-target sampling on high-degree islands.
struct LargestIslandStats {
    IslandId id = 0;
    bool valid = false;
    std::size_t polygonCount = 0;
    double polygonShare = 0.0;
    std::size_t outgoingTraversalCount = 0;
    std::size_t incomingTraversalCount = 0;
    std::size_t distinctOutgoingNeighbors = 0;
    std::size_t distinctIncomingNeighbors = 0;
    std::size_t incidentCorridorCount = 0;
    std::size_t representativeCount = 0;
    bool suppressed = false;
    float massScore = 0.0f;
};

// Directed reachability from the largest island ("mainland"). Forward reachability answers
// what the mainland can reach; reverse reachability answers what can return to it. Mutual
// reachability is their intersection. Satellite shares use a denominator that excludes
// mainland geometry so a dominant mainland cannot hide satellite disconnection.
struct ReachabilityStats {
    IslandId mainlandId = 0;
    bool valid = false;
    std::size_t forwardReachableIslands = 0;
    std::size_t forwardReachablePolygons = 0;
    double forwardReachablePolygonShare = 0.0;
    std::size_t reverseReachableIslands = 0;
    std::size_t reverseReachablePolygons = 0;
    double reverseReachablePolygonShare = 0.0;
    std::size_t mutualReachableIslands = 0;
    std::size_t mutualReachablePolygons = 0;
    double mutualReachablePolygonShare = 0.0;
    std::size_t unreachableIslands = 0;
    std::size_t unreachablePolygons = 0;
    double unreachablePolygonShare = 0.0;
    double forwardSatellitePolygonShare = 0.0;
    double reverseSatellitePolygonShare = 0.0;
    double mutualSatellitePolygonShare = 0.0;
    std::size_t unsuppressedIslandCount = 0;
    std::size_t unsuppressedPolygonCount = 0;
    std::size_t unreachableSuppressedIslands = 0;
    std::size_t unreachableSuppressedPolygons = 0;
};

enum class DirectionOffenseReason : std::uint8_t {
    LimitViolationAB,
    LimitViolationBA,
    MissingAllowedAB,
    MissingAllowedBA,
    NoDirection,
    SelfEdge,
    InvalidIslandRef,
    NonFiniteGeometry,
    InconsistentStoredDistance,
    AdjacencyMismatch,
    ExactDuplicateGeometry
};

// One bounded structural example. Deltas are stored-geometry measurements; whether each
// missing direction is blocked by geometry or by outbound policy is counted separately in
// DirectionValidityStats so examples stay small.
struct DirectionOffense {
    std::uint32_t edgeIndex = 0;
    IslandId islandA = 0;
    IslandId islandB = 0;
    float horizontalDistance = 0.0f;
    float verticalDeltaAB = 0.0f;
    bool traversableAB = false;
    bool traversableBA = false;
    DirectionOffenseReason reason = DirectionOffenseReason::NoDirection;
};

// Structural validation of the stored symmetric-first corridor model against the effective
// build capability and outbound policy. For valid stored graphs every violation count is
// zero. A one-way edge is not a defect on its own: missing directions blocked by geometry
// or by policy are counted separately from unexplained missing directions.
struct DirectionValidityStats {
    static constexpr std::size_t kMaxExamples = 8;
    std::size_t corridorCount = 0;
    std::size_t bidirectionalCount = 0;
    std::size_t oneWayABOnlyCount = 0;
    std::size_t oneWayBAOnlyCount = 0;
    std::size_t noDirectionCount = 0;
    std::size_t directedTraversalCount = 0;
    std::size_t selfEdgeCount = 0;
    std::size_t invalidIslandRefCount = 0;
    std::size_t nonFiniteGeometryCount = 0;
    std::size_t inconsistentStoredDistanceCount = 0;
    std::size_t adjacencyMismatchCount = 0;
    std::size_t duplicateAdjacencyCount = 0;
    std::size_t exactDuplicateGeometryCount = 0;
    std::size_t limitViolationABCount = 0;
    std::size_t limitViolationBACount = 0;
    std::size_t missingAllowedABCount = 0;
    std::size_t missingAllowedBACount = 0;
    std::size_t geometryBlockedABCount = 0;
    std::size_t geometryBlockedBACount = 0;
    std::size_t policyBlockedABCount = 0;
    std::size_t policyBlockedBACount = 0;
    float maxExcessHorizontal = 0.0f;
    float maxExcessUp = 0.0f;
    float maxExcessDown = 0.0f;
    std::vector<DirectionOffense> examples;
};

// Effective build capability and density policy as resolved for the reported graph.
// Explicit meter values are reported as-is; zero-valued tunables fall back to the derived
// autoscaled values shown here so a report reader never has to re-derive ratios.
struct EffectiveBuildSettings {
    float maxHorizontalGap = 0.0f;
    float maxVerticalGapUp = 0.0f;
    float maxVerticalGapDown = 0.0f;
    bool symmetricLimits = false;
    float maxTraversalExtent = 0.0f;
    int queryMaxNodes = 0;
    bool outboundFilterPresent = false;
    bool massAwareEnabled = false;
    bool suppressSmallIslands = false;
    float suppressedIslandPercent = 0.0f;
    bool boundaryDeduplicationEnabled = false;
    float boundaryDeduplicationCellSize = 0.0f;
    float verticalCollapseWindow = 0.0f;
    bool representativeReductionEnabled = false;
    float representativeCellSize = 0.0f;
    int representativeDirectionBuckets = 0;
    std::uint32_t minRepresentativesPerIsland = 0;
    std::uint32_t maxRepresentativesPerIsland = 0;
    bool pairScanSuppressionEnabled = false;
    bool pairScanSuppressionActive = false;
    float pairScanSuppressionCellSize = 0.0f;
    bool shortGapRecoveryEnabled = false;
    float shortGapRecoveryGap = 0.0f;
    bool candidateDeduplicationEnabled = false;
    float candidateDeduplicationCellSize = 0.0f;
    bool localPruningEnabled = false;
    float localPruningRadius = 0.0f;
    bool globalPruningEnabled = false;
    float globalPruningRadius = 0.0f;
    bool spannerPruningEnabled = false;
    float spannerPathRatio = 0.0f;
    bool distinctTargetReserveEnabled = false;
    std::uint32_t minDistinctTargetsPerIsland = 0;
    std::uint32_t maxDistinctTargetsPerIsland = 0;
};

struct BuildStats {
    TimingStats timings;
    BoundaryStats boundaries;
    QueryStats queries;
    CandidateStats candidates;
    std::size_t islandCount = 0;
    std::size_t polygonCount = 0;
    std::size_t smallIslandsSuppressed = 0;
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
    LargestIslandStats largestIsland;
    ReachabilityStats reachability;
    DirectionValidityStats directionValidity;
    EffectiveBuildSettings effectiveSettings;
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
