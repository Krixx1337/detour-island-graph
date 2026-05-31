#include <detour_island_graph/IslandGraphBuilder.h>

#include "IslandGraphBuilderInternal.h"

namespace detour_island_graph {

BuildResult IslandGraphBuilder::build(const dtNavMesh& navMesh, const BuildConfig& config) const {
    BuildResult result;
    const detail::Clock::time_point totalStart = detail::Clock::now();
    if (!detail::validate(config, result.message)) {
        result.status = BuildStatus::InvalidConfiguration;
        result.stats.timings.totalMs = detail::elapsedMilliseconds(totalStart);
        return result;
    }

    const detail::Clock::time_point floodFillStart = detail::Clock::now();
    detail::floodFill(navMesh, result.graph);
    result.stats.timings.floodFillMs = detail::elapsedMilliseconds(floodFillStart);
    result.stats.islandCount = result.graph.islands().size();
    for (const Island& island : result.graph.islands()) {
        result.stats.polygonCount += island.polygons.size();
    }

    const detail::Clock::time_point massScoringStart = detail::Clock::now();
    detail::calculateMassScores(result.graph, config);
    result.stats.timings.massScoringMs = detail::elapsedMilliseconds(massScoringStart);

    result.status = detail::discoverLinks(navMesh, result.graph, config, result.stats, result.message);
    detail::calculateGraphHealthStats(result.graph, result.stats);

    result.stats.timings.totalMs = detail::elapsedMilliseconds(totalStart);
    return result;
}

} // namespace detour_island_graph
