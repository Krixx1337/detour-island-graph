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

inline bool cancellationRequested(const BuildOptions& options) {
    return options.shouldCancel && options.shouldCancel();
}

struct IslandGraphAccess {
    static std::vector<Island>& islands(IslandGraph& graph) {
        return graph.m_islands;
    }

    static std::unordered_map<dtPolyRef, IslandId>& polygonToIsland(IslandGraph& graph) {
        return graph.m_polygonToIsland;
    }

    static std::vector<Edge>& edges(IslandGraph& graph) {
        return graph.m_edges;
    }

    static void rebuildAdjacency(IslandGraph& graph) {
        graph.rebuildAdjacency();
    }
};

bool validate(const BuildConfig& config, std::string& message);
BuildStatus floodFill(
    const dtNavMesh& navMesh,
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options);
BuildStatus calculateMassScores(
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options);
BuildStatus discoverLinks(
    const dtNavMesh& navMesh,
    IslandGraph& graph,
    const BuildConfig& config,
    const BuildOptions& options,
    BuildStats& stats,
    std::string& message);
BuildStatus calculateGraphHealthStats(
    const IslandGraph& graph,
    const BuildOptions& options,
    BuildStats& stats);

} // namespace detour_island_graph::detail
