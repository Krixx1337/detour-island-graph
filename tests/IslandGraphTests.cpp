#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <detour_island_graph/IslandGraph.h>
#include <detour_island_graph/IslandGraphBuilder.h>
#include <detour_island_graph/IslandGraphPathfinder.h>
#include <detour_island_graph/IslandGraphSerializer.h>

#include <DetourAlloc.h>
#include <DetourNavMeshBuilder.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

namespace {

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
    REQUIRE(dtCreateNavMeshData(&params, &data, &dataSize));

    std::unique_ptr<dtNavMesh, NavMeshDeleter> navMesh(dtAllocNavMesh());
    REQUIRE(navMesh != nullptr);
    REQUIRE(dtStatusSucceed(navMesh->init(data, dataSize, DT_TILE_FREE_DATA)));
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
    REQUIRE(dtCreateNavMeshData(&params, &data, &dataSize));

    std::unique_ptr<dtNavMesh, NavMeshDeleter> navMesh(dtAllocNavMesh());
    REQUIRE(navMesh != nullptr);
    REQUIRE(dtStatusSucceed(navMesh->init(data, dataSize, DT_TILE_FREE_DATA)));
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
    REQUIRE(dtCreateNavMeshData(&params, &data, &dataSize));
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
    REQUIRE(navMesh != nullptr);
    REQUIRE(dtStatusSucceed(navMesh->init(&params)));

    int firstSize = 0;
    unsigned char* first = buildTileData(0, 1, 2, firstSize);
    REQUIRE(dtStatusSucceed(navMesh->addTile(first, firstSize, DT_TILE_FREE_DATA, 0, nullptr)));

    int secondSize = 0;
    unsigned char* second = buildTileData(1, 3, 0, secondSize);
    REQUIRE(dtStatusSucceed(navMesh->addTile(second, secondSize, DT_TILE_FREE_DATA, 0, nullptr)));
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

using namespace detour_island_graph;

TEST_CASE("Empty graph initialization") {
    IslandGraph emptyGraph;
    CHECK(emptyGraph.empty());
    CHECK(emptyGraph.findIsland(0) == nullptr);
    CHECK(!emptyGraph.findIslandForPolygon(42).has_value());
}

TEST_CASE("Graph with valid island") {
    IslandGraph graph({makeIsland(0, {42, 43})});
    CHECK_FALSE(graph.empty());
    REQUIRE(graph.findIsland(0) != nullptr);
    CHECK(graph.findIsland(0)->polygons.size() == 2);
    CHECK(graph.findIsland(1) == nullptr);
    CHECK(graph.findIslandForPolygon(42) == 0);
    CHECK(!graph.findIslandForPolygon(99).has_value());
}

TEST_CASE("Graph rejects inconsistent island identifiers") {
    IslandGraph inconsistentGraph({makeIsland(7, {42})});
    CHECK(inconsistentGraph.findIsland(0) == nullptr);
}

TEST_CASE("Default config and path result") {
    const BuildConfig defaultConfig;
    CHECK(defaultConfig.gapDiscovery.maxHorizontalGap > 0.0f);
    const PathResult defaultPathResult;
    CHECK(defaultPathResult.status == PathStatus::NoPath);
}

TEST_CASE("Pathfinder basic routing") {
    const Link direct{0, 2, {0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}, 100.0f, 0.0f};
    const Link firstHop{0, 1, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link secondHop{1, 2, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    IslandGraph routeGraph({
        makeIsland(0, {}, {direct, firstHop}),
        makeIsland(1, {}, {secondHop}),
        makeIsland(2)});
    const IslandGraphPathfinder pathfinder;
    const PathResult path = pathfinder.findPath(
        routeGraph, 0, 2, {0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f});
    REQUIRE(path.status == PathStatus::Success);
    CHECK(path.links.size() == 2);
    CHECK(path.totalCost == 3.0f);
}

TEST_CASE("Pathfinder filtered path") {
    const Link direct{0, 2, {0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}, 100.0f, 0.0f};
    const Link firstHop{0, 1, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link secondHop{1, 2, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    IslandGraph routeGraph({
        makeIsland(0, {}, {direct, firstHop}),
        makeIsland(1, {}, {secondHop}),
        makeIsland(2)});
    const IslandGraphPathfinder pathfinder;
    const PathResult filteredPath = pathfinder.findPath(
        routeGraph, 0, 2, {0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, {},
        [](const Link& link) { return link.toIsland != 1; });
    CHECK(filteredPath.links.size() == 1);
}

TEST_CASE("Pathfinder edge cases") {
    const Link direct{0, 2, {0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}, 100.0f, 0.0f};
    const Link firstHop{0, 1, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link secondHop{1, 2, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    IslandGraph routeGraph({
        makeIsland(0, {}, {direct, firstHop}),
        makeIsland(1, {}, {secondHop}),
        makeIsland(2)});
    const IslandGraphPathfinder pathfinder;
    CHECK(pathfinder.findPath(routeGraph, 0, 0, {}, {}).status == PathStatus::SameIsland);
    CHECK(pathfinder.findPath(routeGraph, 0, 9, {}, {}).status == PathStatus::InvalidIsland);
}

TEST_CASE("Pathfinder custom costs use an admissible search order") {
    const Link expensiveHop{0, 2, {}, {}, 10.0f, 0.0f};
    const Link cheapHop{0, 1, {}, {100.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link bridge{1, 2, {100.0f, 0.0f, 0.0f}, {}, 1.0f, 0.0f};
    const Link goal{2, 3, {}, {}, 1.0f, 0.0f};
    IslandGraph routeGraph({
        makeIsland(0, {}, {expensiveHop, cheapHop}),
        makeIsland(1, {}, {bridge}),
        makeIsland(2, {}, {goal}),
        makeIsland(3)});
    const IslandGraphPathfinder pathfinder;
    const PathResult path = pathfinder.findPath(
        routeGraph, 0, 3, {}, {},
        [](const Link& link) { return link.horizontalDistance; });
    REQUIRE(path.status == PathStatus::Success);
    CHECK(path.links.size() == 3);
    CHECK(path.totalCost == 3.0f);
}

TEST_CASE("Builder with disconnected navmesh") {
    const auto navMesh = buildDisconnectedNavMesh();
    BuildConfig buildConfig;
    buildConfig.gapDiscovery.maxHorizontalGap = 3.0f;
    buildConfig.gapDiscovery.maxVerticalGapUp = 2.0f;
    buildConfig.gapDiscovery.maxVerticalGapDown = 4.0f;
    buildConfig.boundaries.deduplicationCellSize = 0.5f;
    buildConfig.density.localPruning.baseRadius = 0.5f;
    const IslandGraphBuilder builder;
    const BuildResult buildResult = builder.build(*navMesh, buildConfig);
    REQUIRE(static_cast<bool>(buildResult));

    SUBCASE("Flood fill preserves disconnected islands") {
        CHECK(buildResult.graph.islands().size() == 3);
    }
    SUBCASE("Stats report counts") {
        CHECK(buildResult.stats.islandCount == 3);
        CHECK(buildResult.stats.polygonCount == 3);
        CHECK(buildResult.stats.boundaries.rawCount == 12);
    }
    SUBCASE("Boundary deduplication stats are ordered") {
        CHECK(buildResult.stats.boundaries.deduplicatedCount <= buildResult.stats.boundaries.rawCount);
    }
    SUBCASE("Spatial query count matches retained boundaries") {
        CHECK(buildResult.stats.queries.count == buildResult.stats.boundaries.deduplicatedCount);
    }
    SUBCASE("Stats report saturated spatial queries") {
        BuildConfig constrainedConfig = buildConfig;
        constrainedConfig.query.maxNearbyPolygons = 1;
        const BuildResult constrainedBuild = builder.build(*navMesh, constrainedConfig);
        REQUIRE(static_cast<bool>(constrainedBuild));
        CHECK(constrainedBuild.stats.queries.capacityHitCount > 0);
    }
    SUBCASE("Candidate deduplication stats are ordered") {
        CHECK(buildResult.stats.candidates.deduplicatedCount <= buildResult.stats.candidates.projectedCount);
    }
    SUBCASE("Builder discovers accepted links") {
        CHECK(buildResult.stats.candidates.acceptedLinkCount > 0);
    }
    SUBCASE("Stats report build timings") {
        CHECK(buildResult.stats.timings.totalMs >= 0.0);
        CHECK(buildResult.stats.timings.floodFillMs >= 0.0);
        CHECK(buildResult.stats.timings.massScoringMs >= 0.0);
        CHECK(buildResult.stats.timings.boundaryExtractionMs >= 0.0);
        CHECK(buildResult.stats.timings.linkDiscoveryMs >= 0.0);
        CHECK(buildResult.stats.timings.pruningMs >= 0.0);
    }
    SUBCASE("Builder discovers forward and reverse gaps") {
        CHECK(hasLink(buildResult.graph, 0, 1));
        CHECK(hasLink(buildResult.graph, 1, 0));
    }
    SUBCASE("Upward gap obeys upward limit") {
        CHECK(!hasLink(buildResult.graph, 1, 2));
    }
    SUBCASE("Downward gap obeys downward limit") {
        CHECK(hasLink(buildResult.graph, 2, 1));
    }
    SUBCASE("Builder rejects same-island links") {
        for (const Island& island : buildResult.graph.islands()) {
            for (const Link& link : island.outgoingLinks) {
                CHECK(link.fromIsland != link.toIsland);
            }
        }
    }
    SUBCASE("Coarse geometric pruning collapses duplicate links") {
        BuildConfig aggressivelyPrunedConfig = buildConfig;
        aggressivelyPrunedConfig.density.localPruning.baseRadius = 10.0f;
        const BuildResult aggressivelyPruned = builder.build(*navMesh, aggressivelyPrunedConfig);
        REQUIRE(static_cast<bool>(aggressivelyPruned));
        CHECK(linkCount(aggressivelyPruned.graph, 0, 1) == 1);
    }
    SUBCASE("Builder rejects invalid configuration") {
        BuildConfig invalidConfig;
        invalidConfig.density.localPruning.baseRadius = -0.01f;
        CHECK(builder.build(*navMesh, invalidConfig).status == BuildStatus::InvalidConfiguration);
    }
}

TEST_CASE("Builder adjacent tiled navmesh") {
    const auto tiledNavMesh = buildAdjacentTiledNavMesh();
    const IslandGraphBuilder builder;
    const BuildResult tiledBuild = builder.build(*tiledNavMesh);
    REQUIRE(static_cast<bool>(tiledBuild));
    CHECK(tiledBuild.graph.islands().size() == 1);
    CHECK(tiledBuild.graph.islands()[0].polygons.size() == 2);
}

TEST_CASE("Builder density tuning") {
    const auto navMesh = buildDisconnectedNavMesh();
    BuildConfig buildConfig;
    buildConfig.gapDiscovery.maxHorizontalGap = 3.0f;
    buildConfig.gapDiscovery.maxVerticalGapUp = 2.0f;
    buildConfig.gapDiscovery.maxVerticalGapDown = 4.0f;
    buildConfig.boundaries.deduplicationCellSize = 0.5f;
    buildConfig.density.localPruning.baseRadius = 0.5f;
    const IslandGraphBuilder builder;
    const BuildResult baseline = builder.build(*navMesh, buildConfig);
    REQUIRE(static_cast<bool>(baseline));

    SUBCASE("Disabled density preserves baseline") {
        BuildConfig disabledDensityConfig = buildConfig;
        disabledDensityConfig.density.localPruning.enableDistanceScaling = false;
        disabledDensityConfig.density.localPruning.distanceScale = 10.0f;
        disabledDensityConfig.density.localPruning.maxRadiusScale = 100.0f;
        const BuildResult result = builder.build(*navMesh, disabledDensityConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount == baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Local pruning can be disabled independently") {
        BuildConfig unprunedConfig = buildConfig;
        unprunedConfig.density.localPruning.enabled = false;
        unprunedConfig.density.localPruning.baseRadius = 0.0f;
        const BuildResult result = builder.build(*navMesh, unprunedConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount >= baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Density radius scale interpolates continuously") {
        BuildConfig densityConfig;
        densityConfig.density.localPruning.enableDistanceScaling = true;
        densityConfig.density.localPruning.distanceScale = 0.25f;
        densityConfig.density.localPruning.maxRadiusScale = 2.0f;
        CHECK(densityConfig.density.localPruning.pruneRadiusScaleFor(2.0f) == 1.5f);
        CHECK(densityConfig.density.localPruning.pruneRadiusScaleFor(8.0f) == 2.0f);
    }
    SUBCASE("Candidate deduplication cell size interpolates continuously") {
        BuildConfig densityConfig;
        densityConfig.density.candidateDeduplication.enabled = true;
        densityConfig.density.candidateDeduplication.cellSizeNear = 4.0f;
        densityConfig.density.candidateDeduplication.cellSizeFar = 10.0f;
        CHECK(densityConfig.density.candidateDeduplication.cellSizeFor(-1.0f, 30.0f) == 4.0f);
        CHECK(densityConfig.density.candidateDeduplication.cellSizeFor(0.0f, 30.0f) == 4.0f);
        CHECK(densityConfig.density.candidateDeduplication.cellSizeFor(15.0f, 30.0f) == 7.0f);
        CHECK(densityConfig.density.candidateDeduplication.cellSizeFor(60.0f, 30.0f) == 10.0f);
    }
    SUBCASE("Coarse candidate deduplication does not increase accepted links") {
        BuildConfig coarseCandidateConfig = buildConfig;
        coarseCandidateConfig.density.candidateDeduplication.enabled = true;
        coarseCandidateConfig.density.candidateDeduplication.cellSizeNear = 10.0f;
        coarseCandidateConfig.density.candidateDeduplication.cellSizeFar = 10.0f;
        const BuildResult result = builder.build(*navMesh, coarseCandidateConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Candidate deduplication can be disabled independently") {
        BuildConfig undeduplicatedConfig = buildConfig;
        undeduplicatedConfig.density.candidateDeduplication.enabled = false;
        undeduplicatedConfig.density.candidateDeduplication.cellSizeNear = 0.0f;
        undeduplicatedConfig.density.candidateDeduplication.cellSizeFar = 0.0f;
        const BuildResult result = builder.build(*navMesh, undeduplicatedConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.deduplicatedCount >= baseline.stats.candidates.deduplicatedCount);
    }
    SUBCASE("Boundary deduplication can be disabled independently") {
        BuildConfig undeduplicatedConfig = buildConfig;
        undeduplicatedConfig.boundaries.deduplicationEnabled = false;
        undeduplicatedConfig.boundaries.deduplicationCellSize = 0.0f;
        const BuildResult result = builder.build(*navMesh, undeduplicatedConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.boundaries.deduplicatedCount == result.stats.boundaries.rawCount);
    }
    SUBCASE("Enabled density does not increase accepted links") {
        BuildConfig densityConfig = buildConfig;
        densityConfig.density.localPruning.enableDistanceScaling = true;
        densityConfig.density.localPruning.distanceScale = 0.25f;
        densityConfig.density.localPruning.maxRadiusScale = 2.0f;
        const BuildResult densityBuild = builder.build(*navMesh, densityConfig);
        REQUIRE(static_cast<bool>(densityBuild));
        CHECK(densityBuild.stats.candidates.acceptedLinkCount <= baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Stronger distance scale does not increase accepted links") {
        BuildConfig densityConfig = buildConfig;
        densityConfig.density.localPruning.enableDistanceScaling = true;
        densityConfig.density.localPruning.distanceScale = 0.25f;
        densityConfig.density.localPruning.maxRadiusScale = 2.0f;
        const BuildResult densityBuild = builder.build(*navMesh, densityConfig);
        REQUIRE(static_cast<bool>(densityBuild));

        BuildConfig strongerDistanceScaleConfig = densityConfig;
        strongerDistanceScaleConfig.density.localPruning.distanceScale = 0.5f;
        const BuildResult result = builder.build(*navMesh, strongerDistanceScaleConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= densityBuild.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Stronger radius cap does not increase accepted links") {
        BuildConfig densityConfig = buildConfig;
        densityConfig.density.localPruning.enableDistanceScaling = true;
        densityConfig.density.localPruning.distanceScale = 0.25f;
        densityConfig.density.localPruning.maxRadiusScale = 2.0f;
        const BuildResult densityBuild = builder.build(*navMesh, densityConfig);
        REQUIRE(static_cast<bool>(densityBuild));

        BuildConfig strongerRadiusCapConfig = densityConfig;
        strongerRadiusCapConfig.density.localPruning.maxRadiusScale = 3.0f;
        const BuildResult result = builder.build(*navMesh, strongerRadiusCapConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= densityBuild.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Rejects negative density distance scale") {
        BuildConfig invalidDensityConfig;
        invalidDensityConfig.density.localPruning.enableDistanceScaling = true;
        invalidDensityConfig.density.localPruning.distanceScale = -0.01f;
        CHECK(builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Rejects invalid candidate deduplication cell sizes") {
        BuildConfig invalidDensityConfig;
        invalidDensityConfig.density.candidateDeduplication.enabled = true;
        invalidDensityConfig.density.candidateDeduplication.cellSizeNear = -0.01f;
        CHECK(builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Rejects density caps below one") {
        BuildConfig invalidDensityConfig;
        invalidDensityConfig.density.localPruning.enableDistanceScaling = true;
        invalidDensityConfig.density.localPruning.maxRadiusScale = 0.99f;
        CHECK(builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Disabled global pruning preserves baseline") {
        BuildConfig disabledGlobalPruningConfig = buildConfig;
        disabledGlobalPruningConfig.density.globalPruning.enabled = false;
        disabledGlobalPruningConfig.density.globalPruning.relativeCellSize = 0.0f;
        const BuildResult result = builder.build(*navMesh, disabledGlobalPruningConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount == baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Enabled global pruning does not increase accepted links") {
        BuildConfig globalPruningConfig = buildConfig;
        globalPruningConfig.density.globalPruning.enabled = true;
        globalPruningConfig.density.globalPruning.relativeCellSize = 2.0f;
        const BuildResult result = builder.build(*navMesh, globalPruningConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Stronger global pruning does not increase accepted links") {
        BuildConfig globalPruningConfig = buildConfig;
        globalPruningConfig.density.globalPruning.enabled = true;
        globalPruningConfig.density.globalPruning.relativeCellSize = 2.0f;
        const BuildResult globalPruningBuild = builder.build(*navMesh, globalPruningConfig);
        REQUIRE(static_cast<bool>(globalPruningBuild));

        BuildConfig strongerGlobalPruningConfig = buildConfig;
        strongerGlobalPruningConfig.density.globalPruning.enabled = true;
        strongerGlobalPruningConfig.density.globalPruning.relativeCellSize = 4.0f;
        const BuildResult result = builder.build(*navMesh, strongerGlobalPruningConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= globalPruningBuild.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Rejects negative global prune relative cell size") {
        BuildConfig invalidGlobalPruningConfig;
        invalidGlobalPruningConfig.density.globalPruning.enabled = true;
        invalidGlobalPruningConfig.density.globalPruning.relativeCellSize = -0.01f;
        CHECK(builder.build(*navMesh, invalidGlobalPruningConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Disabled spanner pruning preserves baseline") {
        BuildConfig disabledSpannerConfig = buildConfig;
        disabledSpannerConfig.density.spannerPruning.enabled = false;
        const BuildResult result = builder.build(*navMesh, disabledSpannerConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount == baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Enabled spanner pruning does not increase accepted links") {
        BuildConfig spannerConfig = buildConfig;
        spannerConfig.density.spannerPruning.enabled = true;
        spannerConfig.density.spannerPruning.pathRatio = 2.0f;
        const BuildResult result = builder.build(*navMesh, spannerConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Rejects spanner path ratio below one") {
        BuildConfig invalidSpannerConfig;
        invalidSpannerConfig.density.spannerPruning.enabled = true;
        invalidSpannerConfig.density.spannerPruning.pathRatio = 0.99f;
        CHECK(builder.build(*navMesh, invalidSpannerConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Rejects negative spanner vertical weight") {
        BuildConfig invalidSpannerConfig;
        invalidSpannerConfig.density.spannerPruning.enabled = true;
        invalidSpannerConfig.density.spannerPruning.verticalWeight = -0.01f;
        CHECK(builder.build(*navMesh, invalidSpannerConfig).status == BuildStatus::InvalidConfiguration);
    }
}

TEST_CASE("Builder mass-aware tuning") {
    const auto variedMassNavMesh = buildVariedMassNavMesh();
    const IslandGraphBuilder builder;

    SUBCASE("Baseline preserves zero scores") {
        const BuildResult geometricMassBuild = builder.build(*variedMassNavMesh);
        REQUIRE(static_cast<bool>(geometricMassBuild));
        CHECK(geometricMassBuild.graph.islands()[0].massScore == 0.0f);
        CHECK(geometricMassBuild.graph.islands()[1].massScore == 0.0f);
    }
    SUBCASE("Target preference and radius scale interpolate continuously") {
        BuildConfig massAwareConfig;
        massAwareConfig.massAware.enabled = true;
        massAwareConfig.massAware.targetPreference = 2.0f;
        massAwareConfig.massAware.lowMassPruneRadiusScale = 1.5f;
        massAwareConfig.massAware.highMassPruneRadiusScale = 0.75f;
        CHECK(massAwareConfig.massAware.targetPreferenceFor(0.5f) == 1.0f);
        CHECK(massAwareConfig.massAware.pruneRadiusScaleFor(0.5f) == 1.125f);
    }
    SUBCASE("Mass-aware scores are normalized and ordered") {
        BuildConfig massAwareConfig;
        massAwareConfig.massAware.enabled = true;
        massAwareConfig.massAware.targetPreference = 2.0f;
        massAwareConfig.massAware.lowMassPruneRadiusScale = 1.5f;
        massAwareConfig.massAware.highMassPruneRadiusScale = 0.75f;
        const BuildResult massAwareBuild = builder.build(*variedMassNavMesh, massAwareConfig);
        REQUIRE(static_cast<bool>(massAwareBuild));
        CHECK(massAwareBuild.graph.islands().size() == 2);
        CHECK(massAwareBuild.graph.islands()[0].massScore > massAwareBuild.graph.islands()[1].massScore);
        CHECK(massAwareBuild.graph.islands()[0].massScore == 1.0f);
        CHECK(massAwareBuild.graph.islands()[1].massScore >= 0.0f);
    }
    SUBCASE("Nearby percentile varies mass score smoothly") {
        BuildConfig massAwareConfig;
        massAwareConfig.massAware.enabled = true;
        massAwareConfig.massAware.targetPreference = 2.0f;
        massAwareConfig.massAware.lowMassPruneRadiusScale = 1.5f;
        massAwareConfig.massAware.highMassPruneRadiusScale = 0.75f;
        const BuildResult massAwareBuild = builder.build(*variedMassNavMesh, massAwareConfig);
        REQUIRE(static_cast<bool>(massAwareBuild));

        BuildConfig nearbyPercentileConfig = massAwareConfig;
        nearbyPercentileConfig.massAware.normalizationPercentile = 0.98f;
        const BuildResult nearbyPercentileBuild = builder.build(*variedMassNavMesh, nearbyPercentileConfig);
        REQUIRE(static_cast<bool>(nearbyPercentileBuild));
        CHECK(std::abs(
            massAwareBuild.graph.islands()[1].massScore -
            nearbyPercentileBuild.graph.islands()[1].massScore) < 0.02f);
    }
    SUBCASE("Rejects invalid mass normalization percentile") {
        const auto navMesh = buildDisconnectedNavMesh();
        BuildConfig invalidMassAwareConfig;
        invalidMassAwareConfig.massAware.normalizationPercentile = 0.0f;
        CHECK(builder.build(*navMesh, invalidMassAwareConfig).status == BuildStatus::InvalidConfiguration);
    }
}

TEST_CASE("Serializer") {
    const Link direct{0, 2, {0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}, 100.0f, 0.0f};
    const Link firstHop{0, 1, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link secondHop{1, 2, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    IslandGraph routeGraph({
        makeIsland(0, {}, {direct, firstHop}),
        makeIsland(1, {}, {secondHop}),
        makeIsland(2)});

    SUBCASE("Round trip preserves graph") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        serialized.seekg(0);
        const SerializationResult roundTrip = IslandGraphSerializer::read(serialized);
        REQUIRE(static_cast<bool>(roundTrip));
        CHECK(roundTrip.graph.islands().size() == 3);
        CHECK(hasLink(roundTrip.graph, 0, 2));
    }
    SUBCASE("Round trip preserves mass score") {
        const auto variedMassNavMesh = buildVariedMassNavMesh();
        BuildConfig massAwareConfig;
        massAwareConfig.massAware.enabled = true;
        massAwareConfig.massAware.targetPreference = 2.0f;
        massAwareConfig.massAware.lowMassPruneRadiusScale = 1.5f;
        massAwareConfig.massAware.highMassPruneRadiusScale = 0.75f;
        const IslandGraphBuilder builder;
        const BuildResult massAwareBuild = builder.build(*variedMassNavMesh, massAwareConfig);
        REQUIRE(static_cast<bool>(massAwareBuild));

        std::stringstream massSerialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(massSerialized, massAwareBuild.graph) == SerializationStatus::Success);
        massSerialized.seekg(0);
        const SerializationResult massRoundTrip = IslandGraphSerializer::read(massSerialized);
        REQUIRE(static_cast<bool>(massRoundTrip));
        CHECK(massRoundTrip.graph.islands()[0].massScore == massAwareBuild.graph.islands()[0].massScore);
    }
    SUBCASE("Rejects truncated stream") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        std::string bytes = serialized.str();
        bytes.resize(7);
        std::stringstream truncated(bytes, std::ios::in | std::ios::binary);
        CHECK(IslandGraphSerializer::read(truncated).status == SerializationStatus::MalformedData);
    }
    SUBCASE("Rejects unsupported version") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        std::string bytes = serialized.str();
        bytes[4] = 3;
        bytes[5] = 0;
        bytes[6] = 0;
        bytes[7] = 0;
        std::stringstream unsupported(bytes, std::ios::in | std::ios::binary);
        CHECK(IslandGraphSerializer::read(unsupported).status == SerializationStatus::UnsupportedVersion);
    }
    SUBCASE("Rejects payloads that exceed the aggregate allocation budget") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        std::string bytes = serialized.str();
        REQUIRE(bytes.size() > 59);
        bytes[56] = 0;
        bytes[57] = 0;
        bytes[58] = static_cast<char>(0xf4);
        bytes[59] = 0;
        std::stringstream oversized(bytes, std::ios::in | std::ios::binary);
        CHECK(IslandGraphSerializer::read(oversized).status == SerializationStatus::MalformedData);
    }
}
