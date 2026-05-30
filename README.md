# DetourIslandGraph

DetourIslandGraph is a small C++17 library for building and querying an island graph
on top of a Recast/Detour `dtNavMesh`.

The library contains a synchronous graph builder, portal-aware pathfinder, and
optional explicit stream serializer.

## Design

- Coordinates are Detour-native: X/Z are horizontal and Y is vertical.
- Links are directed because upward and downward traversal limits may differ.
- `dtPolyRef` values refer to the source navmesh. Keep that navmesh topology stable
  while using its graph.
- The library contains no engine integration, threading, cache policy, coordinate
  conversion, or third-party math dependency.

## Optional Mass-Aware Tuning

The geometric baseline is the default. Applications may enable optional continuous
mass-aware tuning through `BuildConfig::massAware`.

Each island receives a normalized `massScore` in `[0, 1]` based on polygon count and
horizontal span. The score is normalized against an interpolated map-relative
percentile with a smooth asymptotic curve, then feeds smooth formulas for candidate
preference and pair-pruning radius. There are no backbone/support buckets, tier
thresholds, or hard cliffs.

```cpp
detour_island_graph::BuildConfig config;
config.massAware.enabled = true;
config.massAware.targetPreference = 2.0f;
config.massAware.lowMassPruneRadiusScale = 1.5f;
config.massAware.highMassPruneRadiusScale = 0.75f;
```

Leave `massAware.enabled` as `false` to retain the minimal geometric behavior.

## Build Stats

Each `BuildResult` includes ephemeral `BuildStats` for integration diagnostics and
benchmarks. Stats report stage timings and pipeline counts such as islands, polygons,
raw and deduplicated boundaries, spatial queries, projected and deduplicated
candidates, and accepted links. They do not affect graph behavior or serialization.

## CMake

Provide Detour as a `Detour` or `RecastNavigation::Detour` CMake target:

```cmake
add_subdirectory(path/to/DetourIslandGraph)
target_link_libraries(your_target PRIVATE
    detour_island_graph::detour_island_graph)
```

When no Detour target or local path is provided, standalone builds fetch the pinned
upstream RecastNavigation `v1.6.0` tag automatically. Disable network bootstrap with:

```text
-DDETOUR_ISLAND_GRAPH_FETCH_DETOUR=OFF
```

Installed packages expose the same target:

```cmake
find_package(detour_island_graph CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE
    detour_island_graph::detour_island_graph)
```

For local development with prebuilt Detour headers, set:

```text
-DDETOUR_ISLAND_GRAPH_DETOUR_INCLUDE_DIR=path/to/detour/include
```

Builder use also requires a Detour implementation. A prebuilt library can be supplied
with:

```text
-DDETOUR_ISLAND_GRAPH_DETOUR_LIBRARY=path/to/detour/library
```

These raw path options are build-tree conveniences. Installed-package consumers
should provide Detour through their own CMake integration.

Enable the standalone tests with:

```text
-DDETOUR_ISLAND_GRAPH_BUILD_TESTS=ON
```

## Serialization

`IslandGraphSerializer` reads and writes an explicit versioned little-endian binary
format through standard streams. It does not manage files, cache identity,
compression, or encryption. The caller is responsible for ensuring serialized graph
data matches the source navmesh.
