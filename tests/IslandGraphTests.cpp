#include <detour_island_graph/IslandGraph.h>
#include <detour_island_graph/IslandGraphBuilder.h>
#include <detour_island_graph/IslandGraphPathfinder.h>
#include <detour_island_graph/IslandGraphSerializer.h>

#include <DetourAlloc.h>
#include <DetourNavMeshBuilder.h>

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

struct NavMeshDeleter {
    void operator()(dtNavMesh* navMesh) const {
        if (navMesh) {
            dtFreeNavMesh(navMesh);
        }
    }
};

std::unique_ptr<dtNavMesh, NavMeshDeleter> buildDisconnectedNavMesh() {
    constexpr unsigned short nullIndex = 0xffff;
    const unsigned short vertices[] = {
        0, 0, 0,  2, 0, 0,  2, 0, 2,  0, 0, 2,
        4, 0, 0,  6, 0, 0,  6, 0, 2,  4, 0, 2,
        8, 3, 0, 10, 3, 0, 10, 3, 2,  8, 3, 2};
    const unsigned short polygons[] = {
        0, 1, 2, 3, nullIndex, nullIndex, nullIndex, nullIndex,
        4, 5, 6, 7, nullIndex, nullIndex, nullIndex, nullIndex,
        8, 9, 10, 11, nullIndex, nullIndex, nullIndex, nullIndex};
    const unsigned short flags[] = {1, 1, 1};
    const unsigned char areas[] = {0, 0, 0};

    dtNavMeshCreateParams params;
    std::memset(&params, 0, sizeof(params));
    params.verts = vertices;
    params.vertCount = 12;
    params.polys = polygons;
    params.polyFlags = flags;
    params.polyAreas = areas;
    params.polyCount = 3;
    params.nvp = 4;
    params.bmax[0] = 10.0f;
    params.bmax[1] = 3.0f;
    params.bmax[2] = 2.0f;
    params.walkableHeight = 2.0f;
    params.walkableRadius = 0.5f;
    params.walkableClimb = 0.5f;
    params.cs = 1.0f;
    params.ch = 1.0f;
    params.buildBvTree = true;

    unsigned char* data = nullptr;
    int dataSize = 0;
    require(dtCreateNavMeshData(&params, &data, &dataSize), "synthetic navmesh tile should build");

    std::unique_ptr<dtNavMesh, NavMeshDeleter> navMesh(dtAllocNavMesh());
    require(navMesh != nullptr, "synthetic navmesh allocation should succeed");
    require(
        dtStatusSucceed(navMesh->init(data, dataSize, DT_TILE_FREE_DATA)),
        "synthetic navmesh initialization should succeed");
    return navMesh;
}

std::unique_ptr<dtNavMesh, NavMeshDeleter> buildVariedMassNavMesh() {
    constexpr unsigned short nullIndex = 0xffff;
    const unsigned short vertices[] = {
        0, 0, 0,  2, 0, 0,  2, 0, 2,  0, 0, 2,
        4, 0, 0,  4, 0, 2,
        8, 0, 0, 10, 0, 0, 10, 0, 2,  8, 0, 2};
    const unsigned short polygons[] = {
        0, 1, 2, 3, nullIndex, 1, nullIndex, nullIndex,
        1, 4, 5, 2, nullIndex, nullIndex, nullIndex, 0,
        6, 7, 8, 9, nullIndex, nullIndex, nullIndex, nullIndex};
    const unsigned short flags[] = {1, 1, 1};
    const unsigned char areas[] = {0, 0, 0};

    dtNavMeshCreateParams params;
    std::memset(&params, 0, sizeof(params));
    params.verts = vertices;
    params.vertCount = 10;
    params.polys = polygons;
    params.polyFlags = flags;
    params.polyAreas = areas;
    params.polyCount = 3;
    params.nvp = 4;
    params.bmax[0] = 10.0f;
    params.bmax[1] = 1.0f;
    params.bmax[2] = 2.0f;
    params.walkableHeight = 2.0f;
    params.walkableRadius = 0.5f;
    params.walkableClimb = 0.5f;
    params.cs = 1.0f;
    params.ch = 1.0f;
    params.buildBvTree = true;

    unsigned char* data = nullptr;
    int dataSize = 0;
    require(dtCreateNavMeshData(&params, &data, &dataSize), "varied-mass navmesh tile should build");

    std::unique_ptr<dtNavMesh, NavMeshDeleter> navMesh(dtAllocNavMesh());
    require(navMesh != nullptr, "varied-mass navmesh allocation should succeed");
    require(
        dtStatusSucceed(navMesh->init(data, dataSize, DT_TILE_FREE_DATA)),
        "varied-mass navmesh initialization should succeed");
    return navMesh;
}

unsigned char* buildTileData(int tileX, unsigned short portalEdge, unsigned short portalSide, int& dataSize) {
    constexpr unsigned short nullIndex = 0xffff;
    const unsigned short vertices[] = {
        0, 0, 0,  2, 0, 0,  2, 0, 2,  0, 0, 2};
    unsigned short polygons[] = {
        0, 1, 2, 3, nullIndex, nullIndex, nullIndex, nullIndex};
    polygons[4 + portalEdge] = static_cast<unsigned short>(DT_EXT_LINK | portalSide);
    const unsigned short flags[] = {1};
    const unsigned char areas[] = {0};

    dtNavMeshCreateParams params;
    std::memset(&params, 0, sizeof(params));
    params.verts = vertices;
    params.vertCount = 4;
    params.polys = polygons;
    params.polyFlags = flags;
    params.polyAreas = areas;
    params.polyCount = 1;
    params.nvp = 4;
    params.tileX = tileX;
    params.bmin[0] = static_cast<float>(tileX * 2);
    params.bmax[0] = params.bmin[0] + 2.0f;
    params.bmax[1] = 1.0f;
    params.bmax[2] = 2.0f;
    params.walkableHeight = 2.0f;
    params.walkableRadius = 0.5f;
    params.walkableClimb = 0.5f;
    params.cs = 1.0f;
    params.ch = 1.0f;
    params.buildBvTree = true;

    unsigned char* data = nullptr;
    require(dtCreateNavMeshData(&params, &data, &dataSize), "synthetic tiled navmesh tile should build");
    return data;
}

std::unique_ptr<dtNavMesh, NavMeshDeleter> buildAdjacentTiledNavMesh() {
    dtNavMeshParams params;
    std::memset(&params, 0, sizeof(params));
    params.tileWidth = 2.0f;
    params.tileHeight = 2.0f;
    params.maxTiles = 2;
    params.maxPolys = 1;

    std::unique_ptr<dtNavMesh, NavMeshDeleter> navMesh(dtAllocNavMesh());
    require(navMesh != nullptr, "synthetic tiled navmesh allocation should succeed");
    require(dtStatusSucceed(navMesh->init(&params)), "synthetic tiled navmesh initialization should succeed");

    int firstSize = 0;
    unsigned char* first = buildTileData(0, 1, 2, firstSize);
    require(
        dtStatusSucceed(navMesh->addTile(first, firstSize, DT_TILE_FREE_DATA, 0, nullptr)),
        "first synthetic navmesh tile should attach");
    int secondSize = 0;
    unsigned char* second = buildTileData(1, 3, 0, secondSize);
    require(
        dtStatusSucceed(navMesh->addTile(second, secondSize, DT_TILE_FREE_DATA, 0, nullptr)),
        "second synthetic navmesh tile should attach");
    return navMesh;
}

bool hasLink(const detour_island_graph::IslandGraph& graph, std::uint32_t from, std::uint32_t to) {
    const detour_island_graph::Island* island = graph.findIsland(from);
    if (!island) {
        return false;
    }
    for (const detour_island_graph::Link& link : island->outgoingLinks) {
        if (link.toIsland == to) {
            return true;
        }
    }
    return false;
}

std::size_t linkCount(const detour_island_graph::IslandGraph& graph, std::uint32_t from, std::uint32_t to) {
    const detour_island_graph::Island* island = graph.findIsland(from);
    if (!island) {
        return 0;
    }
    std::size_t count = 0;
    for (const detour_island_graph::Link& link : island->outgoingLinks) {
        if (link.toIsland == to) {
            ++count;
        }
    }
    return count;
}

detour_island_graph::Island makeIsland(
    std::uint32_t id,
    std::vector<dtPolyRef> polygons = {},
    std::vector<detour_island_graph::Link> links = {}) {
    detour_island_graph::Island island;
    island.id = id;
    island.polygons = std::move(polygons);
    island.outgoingLinks = std::move(links);
    return island;
}

} // namespace

int main() {
    using namespace detour_island_graph;

    IslandGraph emptyGraph;
    require(emptyGraph.empty(), "new graph should be empty");
    require(emptyGraph.findIsland(0) == nullptr, "empty graph should not resolve an island");
    require(!emptyGraph.findIslandForPolygon(42).has_value(), "unknown polygon should not resolve");

    IslandGraph graph({makeIsland(0, {42, 43})});

    require(!graph.empty(), "graph with an island should not be empty");
    require(graph.findIsland(0) != nullptr, "known island should resolve");
    require(graph.findIsland(0)->polygons.size() == 2, "resolved island should preserve polygons");
    require(graph.findIsland(1) == nullptr, "out-of-range island should not resolve");
    require(graph.findIslandForPolygon(42) == 0, "known polygon should resolve");
    require(!graph.findIslandForPolygon(99).has_value(), "unknown polygon should remain unresolved");

    IslandGraph inconsistentGraph({makeIsland(7, {42})});
    require(inconsistentGraph.findIsland(0) == nullptr, "graph should reject inconsistent island identifiers");

    const BuildConfig defaultConfig;
    require(defaultConfig.maxHorizontalGap > 0.0f, "default config should allow geometric discovery");

    const PathResult defaultPathResult;
    require(defaultPathResult.status == PathStatus::NoPath, "default path result should report no path");

    const Link direct{0, 2, {0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}, 100.0f, 0.0f};
    const Link firstHop{0, 1, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link secondHop{1, 2, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    IslandGraph routeGraph({
        makeIsland(0, {}, {direct, firstHop}),
        makeIsland(1, {}, {secondHop}),
        makeIsland(2)});
    const IslandGraphPathfinder pathfinder;
    const PathResult path = pathfinder.findPath(
        routeGraph,
        0,
        2,
        {0.0f, 0.0f, 0.0f},
        {3.0f, 0.0f, 0.0f});
    require(path.status == PathStatus::Success, "pathfinder should produce a route");
    require(path.links.size() == 2, "pathfinder should include within-island approach distance");
    require(path.totalCost == 3.0f, "pathfinder should report total route cost");

    const PathResult filteredPath = pathfinder.findPath(
        routeGraph,
        0,
        2,
        {0.0f, 0.0f, 0.0f},
        {3.0f, 0.0f, 0.0f},
        {},
        [](const Link& link) { return link.toIsland != 1; });
    require(filteredPath.links.size() == 1, "path filter should remove disallowed links");

    require(
        pathfinder.findPath(routeGraph, 0, 0, {}, {}).status == PathStatus::SameIsland,
        "same-island query should be explicit");
    require(
        pathfinder.findPath(routeGraph, 0, 9, {}, {}).status == PathStatus::InvalidIsland,
        "invalid island query should be explicit");

    const auto navMesh = buildDisconnectedNavMesh();
    BuildConfig buildConfig;
    buildConfig.maxHorizontalGap = 3.0f;
    buildConfig.maxVerticalGapUp = 2.0f;
    buildConfig.maxVerticalGapDown = 4.0f;
    buildConfig.boundaryDeduplicationCellSize = 0.5f;
    buildConfig.linkDeduplicationCellSize = 0.5f;
    const IslandGraphBuilder builder;
    const BuildResult buildResult = builder.build(*navMesh, buildConfig);
    require(static_cast<bool>(buildResult), "builder should process a valid synthetic navmesh");
    require(buildResult.graph.islands().size() == 3, "flood fill should preserve disconnected islands");
    require(buildResult.stats.islandCount == 3, "stats should report island count");
    require(buildResult.stats.polygonCount == 3, "stats should report polygon count");
    require(buildResult.stats.rawBoundaryCount == 12, "stats should report raw boundary count");
    require(
        buildResult.stats.deduplicatedBoundaryCount <= buildResult.stats.rawBoundaryCount,
        "boundary deduplication stats should be ordered");
    require(
        buildResult.stats.spatialQueryCount == buildResult.stats.deduplicatedBoundaryCount,
        "stats should report one spatial query per retained boundary");
    require(
        buildResult.stats.deduplicatedCandidateCount <= buildResult.stats.projectedCandidateCount,
        "candidate deduplication stats should be ordered");
    require(buildResult.stats.acceptedLinkCount > 0, "stats should report accepted links");
    require(buildResult.stats.totalBuildMs >= 0.0, "stats should report total build timing");
    require(buildResult.stats.floodFillMs >= 0.0, "stats should report flood-fill timing");
    require(buildResult.stats.massScoringMs >= 0.0, "stats should report mass-scoring timing");
    require(buildResult.stats.boundaryExtractionMs >= 0.0, "stats should report boundary timing");
    require(buildResult.stats.linkDiscoveryMs >= 0.0, "stats should report discovery timing");
    require(buildResult.stats.pruningMs >= 0.0, "stats should report pruning timing");
    require(hasLink(buildResult.graph, 0, 1), "builder should discover the first forward gap");
    require(hasLink(buildResult.graph, 1, 0), "builder should discover the first reverse gap");
    require(!hasLink(buildResult.graph, 1, 2), "upward gap should obey upward limit");
    require(hasLink(buildResult.graph, 2, 1), "downward gap should obey downward limit");
    for (const Island& island : buildResult.graph.islands()) {
        for (const Link& link : island.outgoingLinks) {
            require(link.fromIsland != link.toIsland, "builder should reject same-island links");
        }
    }

    BuildConfig invalidConfig;
    invalidConfig.linkDeduplicationCellSize = 0.0f;
    require(
        builder.build(*navMesh, invalidConfig).status == BuildStatus::InvalidConfiguration,
        "builder should reject invalid configuration");

    const auto tiledNavMesh = buildAdjacentTiledNavMesh();
    const BuildResult tiledBuild = builder.build(*tiledNavMesh);
    require(static_cast<bool>(tiledBuild), "builder should process tiled navmesh");
    require(tiledBuild.graph.islands().size() == 1, "flood fill should follow cross-tile Detour links");
    require(tiledBuild.graph.islands()[0].polygons.size() == 2, "cross-tile island should contain both polygons");

    BuildConfig aggressivelyPrunedConfig = buildConfig;
    aggressivelyPrunedConfig.linkDeduplicationCellSize = 10.0f;
    const BuildResult aggressivelyPruned = builder.build(*navMesh, aggressivelyPrunedConfig);
    require(static_cast<bool>(aggressivelyPruned), "builder should support coarse geometric pruning");
    require(
        linkCount(aggressivelyPruned.graph, 0, 1) == 1,
        "coarse geometric pruning should collapse duplicate links for one island pair");

    BuildConfig disabledDensityConfig = buildConfig;
    disabledDensityConfig.density.enabled = false;
    disabledDensityConfig.density.distanceScale = 10.0f;
    disabledDensityConfig.density.maxRadiusScale = 100.0f;
    const BuildResult disabledDensityBuild = builder.build(*navMesh, disabledDensityConfig);
    require(static_cast<bool>(disabledDensityBuild), "disabled density tuning should remain valid");
    require(
        disabledDensityBuild.stats.acceptedLinkCount == buildResult.stats.acceptedLinkCount,
        "disabled density tuning should preserve baseline accepted links");

    BuildConfig densityConfig = buildConfig;
    densityConfig.density.enabled = true;
    densityConfig.density.distanceScale = 0.25f;
    densityConfig.density.maxRadiusScale = 2.0f;
    require(
        densityConfig.density.pruneRadiusScaleFor(2.0f) == 1.5f,
        "density radius scale should grow continuously with link distance");
    require(
        densityConfig.density.pruneRadiusScaleFor(8.0f) == 2.0f,
        "density radius scale should obey its smooth cap");
    const BuildResult densityBuild = builder.build(*navMesh, densityConfig);
    require(static_cast<bool>(densityBuild), "density-aware builder should process valid configuration");
    require(
        densityBuild.stats.acceptedLinkCount <= buildResult.stats.acceptedLinkCount,
        "enabling density tuning should not increase accepted links");

    BuildConfig strongerDensityConfig = densityConfig;
    strongerDensityConfig.density.distanceScale = 0.5f;
    strongerDensityConfig.density.maxRadiusScale = 3.0f;
    const BuildResult strongerDensityBuild = builder.build(*navMesh, strongerDensityConfig);
    require(static_cast<bool>(strongerDensityBuild), "stronger density tuning should remain valid");
    require(
        strongerDensityBuild.stats.acceptedLinkCount <= densityBuild.stats.acceptedLinkCount,
        "stronger density tuning should not increase accepted links");

    BuildConfig invalidDensityConfig;
    invalidDensityConfig.density.distanceScale = -0.01f;
    require(
        builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration,
        "builder should reject negative density distance scale");
    invalidDensityConfig = {};
    invalidDensityConfig.density.maxRadiusScale = 0.99f;
    require(
        builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration,
        "builder should reject density caps below one");

    const auto variedMassNavMesh = buildVariedMassNavMesh();
    const BuildResult geometricMassBuild = builder.build(*variedMassNavMesh);
    require(static_cast<bool>(geometricMassBuild), "baseline builder should process varied island mass");
    require(
        geometricMassBuild.graph.islands()[0].massScore == 0.0f &&
        geometricMassBuild.graph.islands()[1].massScore == 0.0f,
        "disabled mass-aware tuning should preserve zero scores");

    BuildConfig massAwareConfig;
    massAwareConfig.massAware.enabled = true;
    massAwareConfig.massAware.targetPreference = 2.0f;
    massAwareConfig.massAware.lowMassPruneRadiusScale = 1.5f;
    massAwareConfig.massAware.highMassPruneRadiusScale = 0.75f;
    require(
        massAwareConfig.massAware.targetPreferenceFor(0.5f) == 1.0f,
        "target preference should interpolate continuously");
    require(
        massAwareConfig.massAware.pruneRadiusScaleFor(0.5f) == 1.125f,
        "prune radius scale should interpolate continuously");
    const BuildResult massAwareBuild = builder.build(*variedMassNavMesh, massAwareConfig);
    require(static_cast<bool>(massAwareBuild), "mass-aware builder should process varied island mass");
    require(massAwareBuild.graph.islands().size() == 2, "varied-mass fixture should contain two islands");
    require(
        massAwareBuild.graph.islands()[0].massScore > massAwareBuild.graph.islands()[1].massScore,
        "larger island should receive a greater continuous mass score");
    require(
        massAwareBuild.graph.islands()[0].massScore <= 1.0f &&
        massAwareBuild.graph.islands()[1].massScore >= 0.0f,
        "mass scores should remain normalized");

    BuildConfig nearbyPercentileConfig = massAwareConfig;
    nearbyPercentileConfig.massAware.normalizationPercentile = 0.98f;
    const BuildResult nearbyPercentileBuild = builder.build(*variedMassNavMesh, nearbyPercentileConfig);
    require(static_cast<bool>(nearbyPercentileBuild), "nearby percentile configuration should remain valid");
    require(
        std::abs(
            massAwareBuild.graph.islands()[1].massScore -
            nearbyPercentileBuild.graph.islands()[1].massScore) < 0.02f,
        "nearby percentile values should vary mass score smoothly");

    BuildConfig invalidMassAwareConfig;
    invalidMassAwareConfig.massAware.normalizationPercentile = 0.0f;
    require(
        builder.build(*navMesh, invalidMassAwareConfig).status == BuildStatus::InvalidConfiguration,
        "builder should reject invalid mass normalization percentile");

    std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
    require(
        IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success,
        "serializer should write graph");
    serialized.seekg(0);
    const SerializationResult roundTrip = IslandGraphSerializer::read(serialized);
    require(static_cast<bool>(roundTrip), "serializer should read graph");
    require(roundTrip.graph.islands().size() == 3, "round trip should preserve islands");
    require(hasLink(roundTrip.graph, 0, 2), "round trip should preserve links");

    std::stringstream massSerialized(std::ios::in | std::ios::out | std::ios::binary);
    require(
        IslandGraphSerializer::write(massSerialized, massAwareBuild.graph) == SerializationStatus::Success,
        "serializer should write mass-aware graph");
    massSerialized.seekg(0);
    const SerializationResult massRoundTrip = IslandGraphSerializer::read(massSerialized);
    require(static_cast<bool>(massRoundTrip), "serializer should read mass-aware graph");
    require(
        massRoundTrip.graph.islands()[0].massScore == massAwareBuild.graph.islands()[0].massScore,
        "round trip should preserve mass score");

    std::string bytes = serialized.str();
    bytes.resize(7);
    std::stringstream truncated(bytes, std::ios::in | std::ios::binary);
    require(
        IslandGraphSerializer::read(truncated).status == SerializationStatus::MalformedData,
        "serializer should reject truncated stream");

    bytes = serialized.str();
    bytes[4] = 3;
    bytes[5] = 0;
    bytes[6] = 0;
    bytes[7] = 0;
    std::stringstream unsupported(bytes, std::ios::in | std::ios::binary);
    require(
        IslandGraphSerializer::read(unsupported).status == SerializationStatus::UnsupportedVersion,
        "serializer should reject unsupported version");

    std::cout << "DetourIslandGraph tests passed\n";
    return EXIT_SUCCESS;
}
