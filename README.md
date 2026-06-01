# DetourIslandGraph

**DetourIslandGraph** is a lightweight C++17 library for building and querying a topological meta-graph over an existing Recast/Detour `dtNavMesh`.

It identifies disconnected navmesh islands, discovers potential gap crossings between them, and supports high-level routing across the resulting graph. This is useful for vertically layered or fragmented maps with cliffs, broken bridges, rooftops, platforms, and other discontinuities.

## Design

- **Responsive, map-agnostic tuning:** Required gap limits describe agent capabilities. Density defaults scale relative to those limits, similar to responsive layout units, rather than assuming fixed map units or dimensions.
- **Continuous density tuning:** Graph-shaping defaults favor proportional scaling and gradual interpolation over hidden fixed thresholds. Hard limits remain explicit when they represent agent capabilities.
- **Granular graph shaping:** Density and pruning stages can be toggled and tuned independently. Applications can retain conservative defaults, request a sparse graph, build an unpruned reference graph, or apply a custom policy.
- **Detour-native integration:** The library operates on `dtNavMesh`, uses Detour query filters, and does not impose an engine-specific vector type or threading model.
- **Portable persistence:** Graphs can be serialized through standard C++ streams.

## Quick Start

```cpp
#include <detour_island_graph/IslandGraphBuilder.h>

detour_island_graph::BuildConfig config(
    20.0f, // maximum horizontal gap
    4.0f,  // maximum climb
    15.0f  // maximum drop
);

detour_island_graph::IslandGraphBuilder builder;
auto result = builder.build(*navMesh, config);

if (!result) {
    std::cerr << result.message << '\n';
}
```

Direct construction uses conservative defaults. For common alternatives:

```cpp
auto sparse = detour_island_graph::BuildConfig::forProfile(
    detour_island_graph::BuildProfile::Sparse, 20.0f, 4.0f, 15.0f);

auto reference = detour_island_graph::BuildConfig::forProfile(
    detour_island_graph::BuildProfile::Unpruned, 20.0f, 4.0f, 15.0f);
```

## Tuning Strategy

Start with a profile and customize only when the resulting graph or build cost requires it.

- Use the conservative defaults when predictable graph shaping matters more than minimum graph size.
- Use the sparse profile when graph size or build cost matters. It retains a focused recovery pass for short gaps that could otherwise be hidden by coarse density reduction.
- Use the unpruned profile as a diagnostic reference, not as a typical production configuration.
- Treat gap limits as movement capabilities. Use density and pruning controls to shape graph size.
- Prefer relative tuning ratios over fixed distances unless a value represents a real movement constraint.
- Adjust one graph-shaping stage at a time. Compare connectivity, accepted-link density, link distribution, and build cost after each change.

Query the high-level graph with `IslandGraphPathfinder`:

```cpp
#include <detour_island_graph/IslandGraphPathfinder.h>

detour_island_graph::IslandGraphPathfinder pathfinder;
auto path = pathfinder.findPath(
    result.graph,
    startIslandId,
    endIslandId,
    startPosition,
    endPosition
);
```

## Customization

The public configuration API lives in:

- `include/detour_island_graph/IslandGraphBuilder.h`
- `include/detour_island_graph/IslandGraphPathfinder.h`
- `include/detour_island_graph/IslandGraphSerializer.h`

Key extension points include:

- Detour polygon include/exclude flags, an optional polygon predicate, and optional outbound-island policy.
- Independent density and pruning controls.
- Optional custom link ranking.
- Optional cooperative build cancellation.
- Optional route-aware path costs, filters, and heuristics.
- Configurable deserialization safety limits.

Use `detour_island_graph::makeVec3(engineVector)` to adapt vector types that expose `.x`, `.y`, and `.z`.

## CMake Integration

```cmake
add_subdirectory(path/to/DetourIslandGraph)
target_link_libraries(your_target PRIVATE detour_island_graph::detour_island_graph)
```

Provide Detour as a `Detour` or `RecastNavigation::Detour` target. Standalone builds can fetch pinned upstream Detour automatically when neither target is available.

Standalone builds enable tests by default. Embedded builds do not. Override this with:

```cmake
-DDETOUR_ISLAND_GRAPH_BUILD_TESTS=ON
```

When using the built-in Detour fetch, enable 64-bit polygon references with:

```cmake
-DDETOUR_ISLAND_GRAPH_DT_POLYREF64=ON
```

## License

MIT
