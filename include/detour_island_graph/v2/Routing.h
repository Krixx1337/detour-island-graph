#pragma once

#include <detour_island_graph/v2/Build.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace detour_island_graph::v2 {

// One directed island-to-island step. `reverse` selects the BA direction
// (b->a); otherwise the step runs AB (a->b). `from`/`to` are the ordered
// takeoff and landing anchors for this step.
struct RouteLink {
    std::size_t crossing = 0; // Index into CompiledGraph::crossings().
    bool reverse = false;
    Anchor from;
    Anchor to;
};

struct RouteCostContext {
    const CompiledGraph& graph;
    IslandId startIsland = 0;
    IslandId endIsland = 0;
};

// On-island transfer cost between two anchored points. Ad-hoc query positions
// arrive with polygon == 0; compiled anchors carry real polygon refs.
// Must return a finite, nonnegative cost. Null means Euclidean distance.
using TransferCost = std::function<float(IslandId island, const Anchor& from, const Anchor& to)>;
// Gap cost for one directed crossing traversal. Must return a finite,
// nonnegative cost; anything else blocks that traversal. Null means the
// crossing's Euclidean endpoint distance.
using CrossingCost =
    std::function<float(const CompiledCrossing& crossing, bool reverse, const RouteCostContext&)>;
using CrossingFilter =
    std::function<bool(const CompiledCrossing& crossing, bool reverse, const RouteCostContext&)>;

struct RouteOptions {
    TransferCost transferCost;
    CrossingCost crossingCost;
    CrossingFilter crossingFilter;
    std::size_t maxExpandedPortals = 0; // Zero means uncapped.
    std::size_t maxQueuedPortals = 0;
    Cancel canceled;
    // Applies only when the corresponding callback is supplied. A custom cost
    // is estimated unless the caller explicitly declares measured/model costs.
    bool transferCostEstimated = true;
    bool crossingCostEstimated = true;
};

enum class RouteStatus : std::uint8_t {
    Success,
    SameIsland,
    NoPath,
    InvalidIsland,
    InvalidInput,
    BudgetExceeded,
    Canceled,
    CallbackFailed,
    OutOfMemory,
    OutOfDomain // Existing island, excluded or outside explored coverage.
};

struct RouteStats {
    std::size_t expandedPortals = 0;
    std::size_t queuedPortals = 0;
    std::size_t peakOpenSetSize = 0;
    // Cost provenance is independent of search ordering. Built-in Euclidean
    // components remain estimated even when another component is custom.
    bool estimatedCost = true;
    bool estimatedTransferCost = true;
    bool estimatedCrossingCost = true;
    bool usedAStar = false;
};

struct Route {
    std::vector<RouteLink> legs;
    float totalCost = 0;
};

struct RouteResult {
    RouteStatus status = RouteStatus::NoPath;
    RouteStats stats;
    std::optional<Route> value; // Engaged only on Success.
};

// Caller-owned scratch reused across queries to avoid reallocations.
// One scratch per concurrent query; never share one across threads.
struct RouteScratch {
    struct State {
        float cost = std::numeric_limits<float>::infinity();
        std::size_t previous = (std::numeric_limits<std::size_t>::max)();
        RouteLink leg;
        bool closed = false;
    };
    struct HeapEntry {
        float bound = 0; // g + h, queue ordering only.
        float cost = 0; // True accumulated g, for stale-entry checks.
        std::size_t portal = 0; // Index into CompiledGraph::traversals().
    };
    std::vector<State> states;
    std::vector<HeapEntry> heap;
};

// Portal-based routing over compiled traversals. Reuses the graph's
// precomputed offsets; no mutable search state lives in the shared graph.
// Same-island queries return SameIsland without searching. Geometric A*
// runs only under fully default costs; any custom cost provider uses
// Dijkstra. Search stops once the remaining bound cannot beat the best
// completed route. No Detour transfer integration or cross-query cache.
RouteResult findRoute(const CompiledGraph& graph, IslandId startIsland, IslandId endIsland,
    Point startPosition, Point endPosition, const RouteOptions& options = {},
    RouteScratch* scratch = nullptr);

} // namespace detour_island_graph::v2
