#include "IslandGraphDiscoveryInternal.h"

#include <memory>

namespace detour_island_graph {
namespace {

struct QueryDeleter {
    void operator()(dtNavMeshQuery* query) const {
        if (query) {
            dtFreeNavMeshQuery(query);
        }
    }
};

} // namespace

namespace detail {

bool validate(const BuildConfig& config, std::string& message) {
    const MassAwareTuning& massAware = config.massAware;
    const DensityTuning& density = config.density;
    const auto require = [&](bool condition, const char* error) {
        if (!condition) {
            message = error;
        }
        return condition;
    };

    if (!require(
            std::isfinite(config.gapDiscovery.maxHorizontalGap) &&
                config.gapDiscovery.maxHorizontalGap > 0.0f,
            "gapDiscovery.maxHorizontalGap must be finite and greater than zero.")) return false;
    if (!require(
            std::isfinite(config.gapDiscovery.maxVerticalGapUp) &&
                config.gapDiscovery.maxVerticalGapUp >= 0.0f,
            "gapDiscovery.maxVerticalGapUp must be finite and non-negative.")) return false;
    if (!require(
            std::isfinite(config.gapDiscovery.maxVerticalGapDown) &&
                config.gapDiscovery.maxVerticalGapDown >= 0.0f,
            "gapDiscovery.maxVerticalGapDown must be finite and non-negative.")) return false;
    if (!require(
            !config.boundaries.deduplicationEnabled ||
                (std::isfinite(config.boundaries.deduplicationCellSize) &&
                 config.boundaries.deduplicationCellSize >= 0.0f &&
                 std::isfinite(config.boundaries.deduplicationCellSizeRatio) &&
                 config.boundaries.deduplicationCellSizeRatio > 0.0f),
            "Enabled boundary deduplication requires a non-negative finite cell size and a positive finite cell-size ratio.")) return false;
    if (!require(
            !config.boundaries.representativeReductionEnabled ||
                (std::isfinite(config.boundaries.representativeCellSize) &&
                 config.boundaries.representativeCellSize >= 0.0f &&
                 std::isfinite(config.boundaries.representativeCellSizeRatio) &&
                 config.boundaries.representativeCellSizeRatio > 0.0f &&
                 config.boundaries.representativeDirectionBuckets > 0 &&
                 std::isfinite(config.boundaries.representativeMassPower) &&
                 config.boundaries.representativeMassPower > 0.0f &&
                 std::isfinite(config.boundaries.representativeBudgetScale) &&
                 config.boundaries.representativeBudgetScale >= 0.0f),
            "Enabled boundary representative reduction requires a non-negative finite cell size, a positive finite cell-size ratio, at least one direction bucket, a positive finite mass power, and a non-negative finite budget scale.")) return false;
    if (!require(
            config.query.maxNodes > 0 && config.query.maxNodes <= 65535,
            "query.maxNodes must be in the range [1, 65535].")) return false;
    if (!require(
            std::isfinite(massAware.normalizationPercentile) &&
                massAware.normalizationPercentile > 0.0f &&
                massAware.normalizationPercentile <= 1.0f,
            "massAware.normalizationPercentile must be finite and in the range (0, 1].")) return false;
    if (!require(
            std::isfinite(massAware.suppressedIslandPercent) &&
                massAware.suppressedIslandPercent >= 0.0f &&
                massAware.suppressedIslandPercent < 1.0f,
            "massAware.suppressedIslandPercent must be finite and in the range [0, 1).")) return false;
    if (!require(
            std::isfinite(massAware.targetPreference) && massAware.targetPreference >= 0.0f,
            "massAware.targetPreference must be finite and non-negative.")) return false;
    if (!require(
            std::isfinite(massAware.targetPreferenceRatio) && massAware.targetPreferenceRatio >= 0.0f,
            "massAware.targetPreferenceRatio must be finite and non-negative.")) return false;
    if (!require(
            std::isfinite(massAware.lowMassPruneRadiusScale) && massAware.lowMassPruneRadiusScale > 0.0f,
            "massAware.lowMassPruneRadiusScale must be finite and greater than zero.")) return false;
    if (!require(
            std::isfinite(massAware.highMassPruneRadiusScale) && massAware.highMassPruneRadiusScale > 0.0f,
            "massAware.highMassPruneRadiusScale must be finite and greater than zero.")) return false;
    if (!require(
            std::isfinite(density.verticalLayerCollapseRatio) &&
                density.verticalLayerCollapseRatio > 0.0f &&
                density.verticalLayerCollapseRatio <= 1.0f,
            "density.verticalLayerCollapseRatio must be finite and in the range (0, 1].")) return false;
    if (!require(
            !density.pairScanSuppression.enabled ||
                (std::isfinite(density.pairScanSuppression.cellSize) &&
                 density.pairScanSuppression.cellSize >= 0.0f &&
                 std::isfinite(density.pairScanSuppression.cellSizeRatio) &&
                 density.pairScanSuppression.cellSizeRatio > 0.0f),
            "Enabled pair-scan suppression requires a non-negative finite cell size and a positive finite cell-size ratio.")) return false;
    if (!require(
            !density.shortGapRecovery.enabled ||
                (std::isfinite(density.shortGapRecovery.maxHorizontalGap) &&
                 density.shortGapRecovery.maxHorizontalGap >= 0.0f &&
                 std::isfinite(density.shortGapRecovery.maxHorizontalGapRatio) &&
                 density.shortGapRecovery.maxHorizontalGapRatio > 0.0f &&
                 density.shortGapRecovery.maxHorizontalGapRatio <= 1.0f),
            "Enabled short-gap recovery requires a non-negative finite gap and a finite gap ratio in the range (0, 1].")) return false;
    if (!require(
            !density.candidateDeduplication.enabled ||
                (std::isfinite(density.candidateDeduplication.cellSize) &&
                 density.candidateDeduplication.cellSize >= 0.0f &&
                 std::isfinite(density.candidateDeduplication.cellSizeRatio) &&
                 density.candidateDeduplication.cellSizeRatio > 0.0f),
            "Enabled candidate deduplication requires a non-negative finite cell size and a positive finite cell-size ratio.")) return false;
    if (!require(
            !density.localPruning.enabled ||
                (std::isfinite(density.localPruning.radius) &&
                 density.localPruning.radius >= 0.0f &&
                 std::isfinite(density.localPruning.radiusRatio) &&
                 density.localPruning.radiusRatio > 0.0f),
            "Enabled local pruning requires a non-negative finite radius and a positive finite radius ratio.")) return false;
    if (!require(
            !density.globalPruning.enabled ||
                (std::isfinite(density.globalPruning.radius) &&
                 density.globalPruning.radius >= 0.0f &&
                 std::isfinite(density.globalPruning.radiusRatio) &&
                 density.globalPruning.radiusRatio > 0.0f &&
                 std::isfinite(density.globalPruning.lowMassRadiusScale) &&
                 density.globalPruning.lowMassRadiusScale > 0.0f &&
                 std::isfinite(density.globalPruning.highMassRadiusScale) &&
                 density.globalPruning.highMassRadiusScale > 0.0f),
            "Enabled global pruning requires a non-negative finite radius, a positive finite radius ratio, and positive finite mass radius scales.")) return false;
    if (!require(
            !density.spannerPruning.enabled ||
                (std::isfinite(density.spannerPruning.pathRatio) &&
                 density.spannerPruning.pathRatio >= 1.0f),
            "Enabled spanner pruning requires a finite path ratio of at least one.")) return false;
    if (!require(
            !density.distinctTargetReserve.enabled ||
                (density.distinctTargetReserve.minTargetsPerIsland <=
                     density.distinctTargetReserve.maxTargetsPerIsland &&
                 std::isfinite(density.distinctTargetReserve.massPower) &&
                 density.distinctTargetReserve.massPower > 0.0f),
            "Enabled distinct target reserve requires min targets no greater than max targets and a positive finite mass power.")) return false;
    return true;
}

BuildStatus discoverLinks(
    const dtNavMesh& navMesh,
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    std::string& message) {
    using namespace discovery;

    if (cancellationRequested(options)) {
        return BuildStatus::Cancelled;
    }
    std::unique_ptr<dtNavMeshQuery, QueryDeleter> query(dtAllocNavMeshQuery());
    if (!query || dtStatusFailed(query->init(&navMesh, config.query.maxNodes))) {
        message = "Failed to initialize dtNavMeshQuery.";
        return BuildStatus::QueryInitializationFailed;
    }

    const Clock::time_point boundaryStart = Clock::now();
    std::vector<Boundary> boundaries;
    BuildStatus status = extractBoundaries(navMesh, graph, config, options, stats, boundaries, message);
    if (status != BuildStatus::Success) {
        return status;
    }
    std::vector<Boundary> representatives;
    status = selectBoundaryRepresentatives(boundaries, graph, config, options, stats, representatives);
    if (status != BuildStatus::Success) {
        return status;
    }
    stats.timings.boundaryExtractionMs = elapsedMilliseconds(boundaryStart);

    std::vector<Link> candidates;
    status = discoverCandidates(
        *query, graph, config, options, boundaries, representatives, stats, candidates, message);
    if (status != BuildStatus::Success) {
        return status;
    }
    return pruneCandidates(graph, config, options, stats, candidates);
}

} // namespace detail
} // namespace detour_island_graph
