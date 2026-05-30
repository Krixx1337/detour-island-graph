# DetourIslandGraph

**DetourIslandGraph** is a lightweight, high-performance C++17 library designed to automatically build and query a **topological meta-graph** on top of your existing Recast/Detour `dtNavMesh`.

In complex 3D games with vertically layered maps (cliffs, broken bridges, teleporters, platforms, rooftops, and dungeons), players and AI agents frequently need to cross gaps where there is no continuous navmesh connectivity. DetourIslandGraph solves this by automatically discovering these gaps, generating traverse-able jump links between disconnected navmesh "islands," and providing ultra-fast hierarchical pathfinding across them.

---

## Why Use DetourIslandGraph?

*   🌐 **Automatic Jump-Link Generation:** Automatically extracts navmesh boundaries, queries nearby geometry, and generates directional link candidates across gaps (falls, leaps, teleports) between separate navmesh islands.
*   ⚡ **Ultra-Fast Hierarchical Pathfinding:** Instead of running expensive, fine-grained A* searches across hundreds of thousands of polygons on huge maps, your AI can perform a lightning-fast search on the sparse "island graph" first, then execute local pathfinding between portals.
*   🎒 **Extremely Lean & Zero-Dependency:** Written in clean C++17. No engine integration, no custom threading models, and no third-party math or vector dependencies. It integrates directly on top of your existing Detour code.
*   📉 **Smart Geometric Density Control:** Includes global spatial occupancy grid pruning and pair-local deduplication to prevent links from exploding on large coastlines or cliffs, keeping A* search branching factors optimal and memory footprint minimal.
*   💾 **Stream Serialization:** Includes a fast, explicit binary serializer to save and load generated island graphs through standard C++ streams.

---

## How It Works

1.  **Extract Islands:** Runs a fast topological flood-fill on the `dtNavMesh` to identify completely separate ground regions (islands).
2.  **Discover Boundaries:** Extracts the open edges of these islands where gaps or cliffs occur.
3.  **Detect Links:** Uses spatial queries on the navmesh to find close landing points across the gaps, generating candidate jump links.
4.  **Prune Redundancy:** Applies geometric density filters and global spatial occupancy grids to keep only the best, most unique link options.
5.  **Hierarchical Pathfinder:** Executes a fast A* search on the resulting sparse island graph.

---

## Quick Start Example

### 1. Build the Island Graph

Building the topological graph is as simple as defining your gap tolerances and passing your Detour navmesh to the builder:

```cpp
#include <detour_island_graph/IslandGraphBuilder.h>

// 1. Configure the builder parameters
detour_island_graph::BuildConfig config;
config.gapDiscovery.maxHorizontalGap = 20.0f;     // Maximum jump distance
config.gapDiscovery.maxVerticalGapUp = 4.0f;      // Maximum climbable/jumpable height
config.gapDiscovery.maxVerticalGapDown = 15.0f;    // Maximum drop height (falls)

// Enable global density pruning to keep the graph clean & sparse
config.density.globalPruning.enabled = true;
config.density.globalPruning.cellSize = 12.0f; // 12-meter global occupancy grid

// 2. Build the graph
detour_island_graph::IslandGraphBuilder builder;
detour_island_graph::BuildResult result = builder.build(*myDetourNavMesh, config);

if (result) {
    // Access the generated graph
    const detour_island_graph::IslandGraph& graph = result.graph;
    std::cout << "Successfully generated " << result.stats.candidates.acceptedLinkCount << " jump links!\n";
}
```

### 2. Query a High-Level Path

Once built, you can query high-level paths across the islands:

```cpp
#include <detour_island_graph/IslandGraphPathfinder.h>

detour_island_graph::IslandGraphPathfinder pathfinder;

// Query a path from a start position to an end position across the graph
detour_island_graph::PathResult path = pathfinder.findPath(
    result.graph,
    startIslandId,
    endIslandId,
    startPositionVec3,
    endPositionVec3
);

if (path.status == detour_island_graph::PathStatus::Success) {
    // Iterate over the meta-links/portals that connect the islands
    for (const detour_island_graph::Link& jumpLink : path.links) {
        std::cout << "Take jump link from (" 
                  << jumpLink.start.x << ", " << jumpLink.start.z << ") to ("
                  << jumpLink.end.x << ", " << jumpLink.end.z << ")\n";
    }
}
```

The default geometric link cost uses A* search with a Euclidean heuristic. Supplying a custom `LinkCost` callback automatically switches to Dijkstra ordering, preserving optimal routes for arbitrary non-negative custom costs at the expense of potentially exploring more portals.

---

## Advanced Configuration

### Global & Local Density Tuning
If your game has large continents, coastlines, or long cliffs, you can enable `density` tuning to prevent the builder from generating hundreds of parallel redundant jump links.

*   `boundaries`: Controls boundary extraction deduplication. Set `deduplicationEnabled = false` to retain every extracted boundary edge, or tune `deduplicationCellSize`.
*   `density.localPruning`: Controls pair-local redundancy pruning. Disable it to retain every discovered link, or enable distance scaling to grow `baseRadius` using `distanceScale` and `maxRadiusScale`.
*   `density.globalPruning`: Controls the optional 3D occupancy grid. Enable it and set `cellSize` to keep only one link start or end point in each global cell.
*   `density.candidateDeduplication`: Controls early candidate deduplication. Disable it to retain every projected candidate, or interpolate from `cellSizeNear` to `cellSizeFar` as links approach `maxHorizontalGap`.
*   `density.spannerPruning`: Controls optional **t-Spanner pruning**. Enable it to discard direct jump links when a multi-hop route is close enough according to `pathRatio`; increase `verticalWeight` when elevation is materially harder than horizontal travel.

### Mass-Aware Tuning
Optionally prefer paths through larger, safer islands (high mass) over tiny, unstable stepping stones (low mass) by enabling `config.massAware.enabled = true`. It calculates a continuous mass score based on polygon count and dimensions to dynamically favor larger islands and adjust pruning tolerances.

### Build Diagnostics
`BuildStats::queries.capacityHitCount` reports how often `queryPolygons()` filled the configured `query.maxNearbyPolygons` buffer. Increase that capacity when the counter is non-zero and missing candidates matter for your mesh.

---

## CMake Integration

Add Detour as a `Detour` or `RecastNavigation::Detour` target in your CMake setup:

```cmake
add_subdirectory(path/to/DetourIslandGraph)
target_link_libraries(your_target PRIVATE detour_island_graph::detour_island_graph)
```

By default, standalone builds will automatically fetch pinned upstream Detour `v1.6.0` headers if not already available in your environment. You can compile the standalone test suite by configuring CMake with `-DDETOUR_ISLAND_GRAPH_BUILD_TESTS=ON`.

For an installed package, define a `Detour` or `RecastNavigation::Detour` target before calling `find_package`:

```cmake
add_subdirectory(path/to/recastnavigation/Detour)
find_package(detour_island_graph CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE detour_island_graph::detour_island_graph)
```

---

## License

This library is licensed under the MIT License.
