#pragma once
#include <detour_island_graph/v2/Build.h>
#include <array>

namespace detour_island_graph::v2 {
enum class HealthIssue : std::uint8_t { Ownership, Metrics, Anchor, Crossing, Duplicate,
    Direction, Adjacency, Metadata, Count };
struct HealthOptions {
    std::size_t maxIslands = 100000, maxCrossings = 1000000, maxTraversals = 2000000;
    std::size_t maxPolygons = 1000000, maxIssueExamples = 16;
    Cancel canceled;
};
struct HealthExample { HealthIssue issue; std::size_t record; };
struct HealthWeight {
    std::size_t islands = 0, polygons = 0;
    std::optional<double> surfaceArea;
};
struct IslandHealth {
    std::size_t incomingNeighbors = 0, outgoingNeighbors = 0;
    std::optional<std::size_t> component;
    bool forward = false, reverse = false;
};
struct GraphHealth {
    std::array<std::size_t, static_cast<std::size_t>(HealthIssue::Count)> issues{};
    std::vector<HealthExample> examples;
    std::vector<IslandHealth> islands;
    std::size_t excluded = 0, unexplored = 0, isolated = 0, zeroIncoming = 0, zeroOutgoing = 0;
    std::size_t bothDirections = 0, oneDirection = 0, neitherDirection = 0, directedTraversals = 0;
    std::vector<HealthWeight> components;
    std::optional<IslandId> referenceIsland;
    HealthWeight included, forward, reverse, mutual;
    bool healthy() const noexcept;
};
struct HealthResult {
    StageStatus status = StageStatus::InvalidInput;
    std::optional<GraphHealth> value;
};
// Opt-in analysis. Size limits bound linear auxiliary storage, not RSS or runtime.
// Success means analysis completed; inspect healthy() for invariant failures.
// Polygon references are checked against graph ownership, not a live mesh.
HealthResult analyzeGraphHealth(const CompiledGraph&, const HealthOptions& = {});
}
