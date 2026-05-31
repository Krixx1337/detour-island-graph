#pragma once

#include <detour_island_graph/IslandGraphBuilder.h>

#include <chrono>
#include <unordered_map>
#include <vector>

namespace detour_island_graph::detail {

using Clock = std::chrono::steady_clock;

inline double elapsedMilliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

struct IslandGraphAccess {
    static std::vector<Island>& islands(IslandGraph& graph) {
        return graph.m_islands;
    }

    static std::unordered_map<dtPolyRef, IslandId>& polygonToIsland(IslandGraph& graph) {
        return graph.m_polygonToIsland;
    }
};

bool validate(const BuildConfig& config, std::string& message);
void floodFill(const dtNavMesh& navMesh, IslandGraph& graph);
void calculateMassScores(IslandGraph& graph, const BuildConfig& config);
BuildStatus discoverLinks(
    const dtNavMesh& navMesh,
    IslandGraph& graph,
    const BuildConfig& config,
    BuildStats& stats,
    std::string& message);
void calculateGraphHealthStats(const IslandGraph& graph, BuildStats& stats);

} // namespace detour_island_graph::detail
