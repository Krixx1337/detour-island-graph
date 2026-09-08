#pragma once

#include <detour_island_graph/v2/Routing.h>
#include <memory>

namespace detour_island_graph::v2 {

struct NativeTransferOptions {
    // All limits except cacheEntries must be positive. Units are navmesh units.
    std::size_t maxQueries = 0;
    std::size_t maxIterations = 0; // Aggregate sliced-search iterations per route.
    int maxNodes = 0; // Detour pool requires 4..65535 nodes.
    int maxCorridor = 0;
    int maxStraightPoints = 0;
    std::size_t cacheEntries = 0; // Zero disables caching; full cache stops inserting.
    float projectionTolerance = 0;
    Cancel canceled;
};

struct NativeTransferStats {
    std::size_t queries = 0;
    std::size_t cacheHits = 0;
    std::size_t cacheMisses = 0;
    std::size_t blocked = 0;
    std::size_t iterations = 0;
    std::size_t peakCorridor = 0;
    std::size_t peakStraightPoints = 0;
    std::size_t cacheEntries = 0;
    double maxProjectionDistance = 0;
};

// Optional module: build with DETOUR_ISLAND_GRAPH_NATIVE_TRANSFERS=ON and a
// consistently compiled DT_VIRTUAL_QUERYFILTER Detour dependency.
// One provider per concurrent query. begin() clears results and cache, retaining
// buffer capacity. Mesh, graph, and callback captures must remain frozen/alive
// until the next begin(). Identity is caller evidence, not a computed mesh hash.
class NativeTransferProvider {
public:
    NativeTransferProvider();
    ~NativeTransferProvider();
    NativeTransferProvider(const NativeTransferProvider&) = delete;
    NativeTransferProvider& operator=(const NativeTransferProvider&) = delete;
    TransferStatus begin(const dtNavMesh& mesh, const CompiledGraph& graph,
        std::uint64_t meshIdentity, const NativeTransferOptions& options);
    TransferStatus checkAnchor(const Anchor& anchor);
    TransferResult evaluate(IslandId island, const Anchor& from, const Anchor& to);
    NativeTransferStats stats() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct NativeRouteResult {
    RouteResult route;
    NativeTransferStats transfers;
};

// Endpoint refs are explicit. Projection is checked within tolerance. Returned
// route anchors remain the graph's original anchors; transfer geometry uses
// their checked projections. Cost is a Detour corridor model, not detail-mesh
// surface distance or a continuous-space optimality guarantee.
NativeRouteResult findNativeRoute(const CompiledGraph& graph, const dtNavMesh& mesh,
    std::uint64_t meshIdentity, Anchor start, Anchor end,
    const NativeTransferOptions& nativeOptions, const RouteOptions& routeOptions = {},
    NativeTransferProvider* provider = nullptr, RouteScratch* scratch = nullptr);

} // namespace detour_island_graph::v2
