#include <detour_island_graph/IslandGraphBuilder.h>

#include "IslandGraphBuilderInternal.h"

namespace detour_island_graph {

BuildConfig BuildConfig::forProfile(
    BuildProfile profile,
    float maxHorizontalGap,
    float maxVerticalGapUp,
    float maxVerticalGapDown) {
    BuildConfig config(maxHorizontalGap, maxVerticalGapUp, maxVerticalGapDown);
    switch (profile) {
    case BuildProfile::Conservative:
        break;
    case BuildProfile::Sparse:
        config.boundaries.representativeReductionEnabled = true;
        config.density.pairScanSuppression.enabled = true;
        config.density.globalPruning.enabled = true;
        config.density.spannerPruning.enabled = true;
        break;
    case BuildProfile::Unpruned:
        config.boundaries.deduplicationEnabled = false;
        config.boundaries.representativeReductionEnabled = false;
        config.density.pairScanSuppression.enabled = false;
        config.density.candidateDeduplication.enabled = false;
        config.density.localPruning.enabled = false;
        config.density.globalPruning.enabled = false;
        config.density.spannerPruning.enabled = false;
        break;
    }
    return config;
}

BuildResult IslandGraphBuilder::build(
    const dtNavMesh& navMesh,
    const BuildConfig& config,
    const BuildOptions& options) const {
    BuildResult result;
    const detail::Clock::time_point totalStart = detail::Clock::now();
    const auto finish = [&]() {
        if (result.status == BuildStatus::Cancelled) {
            result.graph = {};
            result.message = "Build cancelled.";
        }
        result.stats.timings.totalMs = detail::elapsedMilliseconds(totalStart);
        return result;
    };
    if (!detail::validate(config, result.message)) {
        result.status = BuildStatus::InvalidConfiguration;
        return finish();
    }
    if (detail::cancellationRequested(options)) {
        result.status = BuildStatus::Cancelled;
        return finish();
    }

    const detail::Clock::time_point floodFillStart = detail::Clock::now();
    result.status = detail::floodFill(navMesh, result.graph, config, options);
    result.stats.timings.floodFillMs = detail::elapsedMilliseconds(floodFillStart);
    if (result.status != BuildStatus::Success) {
        return finish();
    }
    result.stats.islandCount = result.graph.islands().size();
    for (const Island& island : result.graph.islands()) {
        result.stats.polygonCount += island.polygons.size();
    }

    const detail::Clock::time_point massScoringStart = detail::Clock::now();
    result.status = detail::calculateMassScores(result.graph, config, options);
    result.stats.timings.massScoringMs = detail::elapsedMilliseconds(massScoringStart);
    if (result.status != BuildStatus::Success) {
        return finish();
    }

    result.status = detail::discoverLinks(navMesh, result.graph, config, options, result.stats, result.message);
    if (result.status != BuildStatus::Success) {
        return finish();
    }
    result.status = detail::calculateGraphHealthStats(result.graph, options, result.stats);

    return finish();
}

} // namespace detour_island_graph
