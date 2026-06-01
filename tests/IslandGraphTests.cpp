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

std::unique_ptr<dtNavMesh, NavMeshDeleter> buildVariedMassNavMesh(unsigned short scale = 1) {
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
    params.bmax[0] = 10.0f * scale;
    params.bmax[1] = 1.0f * scale;
    params.bmax[2] = 2.0f * scale;
    params.walkableHeight = 2.0f * scale;
    params.walkableRadius = 0.5f * scale;
    params.walkableClimb = 0.5f * scale;
    params.cs = 1.0f * scale;
    params.ch = 1.0f * scale;
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

bool haveSameLinks(
    const detour_island_graph::IslandGraph& lhs,
    const detour_island_graph::IslandGraph& rhs) {
    if (lhs.islands().size() != rhs.islands().size()) {
        return false;
    }
    for (std::size_t islandIndex = 0; islandIndex < lhs.islands().size(); ++islandIndex) {
        const auto& lhsLinks = lhs.islands()[islandIndex].outgoingLinks;
        const auto& rhsLinks = rhs.islands()[islandIndex].outgoingLinks;
        if (lhsLinks.size() != rhsLinks.size()) {
            return false;
        }
        for (std::size_t linkIndex = 0; linkIndex < lhsLinks.size(); ++linkIndex) {
            const auto& lhsLink = lhsLinks[linkIndex];
            const auto& rhsLink = rhsLinks[linkIndex];
            if (lhsLink.fromIsland != rhsLink.fromIsland ||
                lhsLink.toIsland != rhsLink.toIsland ||
                lhsLink.start.x != rhsLink.start.x ||
                lhsLink.start.y != rhsLink.start.y ||
                lhsLink.start.z != rhsLink.start.z ||
                lhsLink.end.x != rhsLink.end.x ||
                lhsLink.end.y != rhsLink.end.y ||
                lhsLink.end.z != rhsLink.end.z) {
                return false;
            }
        }
    }
    return true;
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

TEST_CASE("Configured gaps and default path result") {
    const BuildConfig defaultConfig(30.0f, 30.0f, 30.0f);
    CHECK(defaultConfig.gapDiscovery.maxHorizontalGap > 0.0f);
    const PathResult defaultPathResult;
    CHECK(defaultPathResult.status == PathStatus::NoPath);
}

TEST_CASE("Vec3 adapter accepts x y z vector types") {
    struct EngineVector {
        double x;
        double y;
        double z;
    };
    const Vec3 value = makeVec3(EngineVector{1.0, 2.0, 3.0});
    CHECK(value.x == 1.0f);
    CHECK(value.y == 2.0f);
    CHECK(value.z == 3.0f);
}

TEST_CASE("Build profiles expose conservative sparse and unpruned defaults") {
    const BuildConfig conservative =
        BuildConfig::forProfile(BuildProfile::Conservative, 30.0f, 30.0f, 30.0f);
    CHECK(conservative.boundaries.deduplicationEnabled);
    CHECK(conservative.density.candidateDeduplication.enabled);
    CHECK(conservative.density.localPruning.enabled);
    CHECK_FALSE(conservative.density.globalPruning.enabled);

    const BuildConfig sparse =
        BuildConfig::forProfile(BuildProfile::Sparse, 30.0f, 30.0f, 30.0f);
    CHECK(sparse.boundaries.representativeReductionEnabled);
    CHECK(sparse.density.pairScanSuppression.enabled);
    CHECK(sparse.density.globalPruning.enabled);
    CHECK(sparse.density.spannerPruning.enabled);

    const BuildConfig unpruned =
        BuildConfig::forProfile(BuildProfile::Unpruned, 30.0f, 30.0f, 30.0f);
    CHECK_FALSE(unpruned.boundaries.deduplicationEnabled);
    CHECK_FALSE(unpruned.density.candidateDeduplication.enabled);
    CHECK_FALSE(unpruned.density.localPruning.enabled);
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
        routeGraph,
        0,
        2,
        {0.0f, 0.0f, 0.0f},
        {3.0f, 0.0f, 0.0f},
        PathOptions{{}, [](const Link& link) { return link.toIsland != 1; }, {}});
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

TEST_CASE("Pathfinder ignores malformed graph links") {
    const Link wrongSource{7, 1, {}, {}, 1.0f, 0.0f};
    const Link invalidTarget{0, 9, {}, {}, 1.0f, 0.0f};
    IslandGraph graph({
        makeIsland(0, {}, {wrongSource, invalidTarget}),
        makeIsland(1)});
    const IslandGraphPathfinder pathfinder;
    CHECK(pathfinder.findPath(graph, 0, 1, {}, {}).status == PathStatus::NoPath);
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
    int heuristicCalls = 0;
    const PathResult path = pathfinder.findPath(
        routeGraph,
        0,
        3,
        {},
        {},
        PathOptions{
            [](const Link& link) { return link.horizontalDistance; },
            {},
            [&heuristicCalls](const Vec3&, const Vec3&) {
                ++heuristicCalls;
                return 0.0f;
            }});
    REQUIRE(path.status == PathStatus::Success);
    CHECK(path.links.size() == 3);
    CHECK(path.totalCost == 3.0f);
    CHECK(heuristicCalls > 0);
}

TEST_CASE("Builder with disconnected navmesh") {
    const auto navMesh = buildDisconnectedNavMesh();
    BuildConfig buildConfig(30.0f, 30.0f, 30.0f);
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
    SUBCASE("Spatial query count matches representative boundaries") {
        CHECK(buildResult.stats.queries.count == buildResult.stats.boundaries.representativeCount);
    }
    SUBCASE("Default outbound policy preserves every boundary") {
        CHECK(buildResult.stats.boundaries.outboundFilteredCount == 0);
    }
    SUBCASE("Spatial queries report complete polygon totals") {
        CHECK(buildResult.stats.queries.nearbyPolygonCount >= buildResult.stats.queries.count);
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
    SUBCASE("Outbound island filtering preserves islands and incoming links") {
        BuildConfig filteredConfig = buildConfig;
        std::size_t filterCallCount = 0;
        filteredConfig.outboundIslandFilter = [&filterCallCount](const Island& island, const IslandGraph&) {
            ++filterCallCount;
            return island.id != 0;
        };
        const BuildResult filtered = builder.build(*navMesh, filteredConfig);
        REQUIRE(static_cast<bool>(filtered));
        CHECK(filtered.graph.islands().size() == buildResult.graph.islands().size());
        CHECK(filterCallCount == filtered.graph.islands().size());
        CHECK(filtered.stats.boundaries.outboundFilteredCount > 0);
        CHECK(filtered.stats.queries.count < buildResult.stats.queries.count);
        CHECK(filtered.graph.islands()[0].outgoingLinks.empty());
        CHECK(hasLink(filtered.graph, 1, 0));
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
        BuildConfig invalidConfig(30.0f, 30.0f, 30.0f);
        invalidConfig.density.localPruning.baseRadius = -0.01f;
        const BuildResult result = builder.build(*navMesh, invalidConfig);
        CHECK(result.status == BuildStatus::InvalidConfiguration);
        CHECK(result.message == "Enabled local pruning requires a non-negative finite base radius and a positive finite base-radius ratio.");
    }
}

TEST_CASE("Builder adjacent tiled navmesh") {
    const auto tiledNavMesh = buildAdjacentTiledNavMesh();
    const IslandGraphBuilder builder;
    const BuildResult tiledBuild =
        builder.build(*tiledNavMesh, BuildConfig(30.0f, 30.0f, 30.0f));
    REQUIRE(static_cast<bool>(tiledBuild));
    CHECK(tiledBuild.graph.islands().size() == 1);
    CHECK(tiledBuild.graph.islands()[0].polygons.size() == 2);
}

TEST_CASE("Builder density tuning") {
    const auto navMesh = buildDisconnectedNavMesh();
    BuildConfig buildConfig(30.0f, 30.0f, 30.0f);
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
        BuildConfig densityConfig(30.0f, 30.0f, 30.0f);
        densityConfig.density.localPruning.enableDistanceScaling = true;
        densityConfig.density.localPruning.distanceScale = 0.25f;
        densityConfig.density.localPruning.maxRadiusScale = 2.0f;
        CHECK(densityConfig.density.localPruning.pruneRadiusScaleFor(2.0f, 30.0f) == 1.5f);
        CHECK(densityConfig.density.localPruning.pruneRadiusScaleFor(8.0f, 30.0f) == 2.0f);
    }
    SUBCASE("Density radius scale supports a horizontal-gap ratio") {
        BuildConfig densityConfig(30.0f, 30.0f, 30.0f);
        densityConfig.density.localPruning.enableDistanceScaling = true;
        densityConfig.density.localPruning.distanceScaleRatio = 1.0f;
        densityConfig.density.localPruning.maxRadiusScale = 3.0f;
        CHECK(densityConfig.density.localPruning.pruneRadiusScaleFor(15.0f, 30.0f) == 1.5f);
        CHECK(densityConfig.density.localPruning.pruneRadiusScaleFor(60.0f, 30.0f) == 3.0f);
    }
    SUBCASE("Candidate deduplication cell size responds to link distance") {
        BuildConfig densityConfig(30.0f, 30.0f, 30.0f);
        densityConfig.density.candidateDeduplication.enabled = true;
        densityConfig.density.candidateDeduplication.nearCellSizeRatio = 0.25f;
        densityConfig.density.candidateDeduplication.farCellSizeRatio = 0.5f;
        CHECK(densityConfig.density.candidateDeduplication.effectiveCellSize(0.0f, 30.0f) == 7.5f);
        CHECK(densityConfig.density.candidateDeduplication.effectiveCellSize(15.0f, 30.0f) == 11.25f);
        CHECK(densityConfig.density.candidateDeduplication.effectiveCellSize(60.0f, 30.0f) == 15.0f);
        densityConfig.density.candidateDeduplication.cellSize = 4.0f;
        CHECK(densityConfig.density.candidateDeduplication.effectiveCellSize(15.0f, 30.0f) == 4.0f);
    }
    SUBCASE("Coarse candidate deduplication does not increase accepted links") {
        BuildConfig coarseCandidateConfig = buildConfig;
        coarseCandidateConfig.density.candidateDeduplication.enabled = true;
        coarseCandidateConfig.density.candidateDeduplication.cellSize = 10.0f;
        const BuildResult result = builder.build(*navMesh, coarseCandidateConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Candidate deduplication can be disabled independently") {
        BuildConfig undeduplicatedConfig = buildConfig;
        undeduplicatedConfig.density.candidateDeduplication.enabled = false;
        undeduplicatedConfig.density.candidateDeduplication.cellSize = 0.0f;
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
    SUBCASE("Pair scan suppression cell size resolves once from the horizontal gap") {
        BuildConfig densityConfig(30.0f, 30.0f, 30.0f);
        densityConfig.density.pairScanSuppression.enabled = true;
        densityConfig.density.pairScanSuppression.cellSizeRatio = 0.25f;
        CHECK(densityConfig.density.pairScanSuppression.effectiveCellSize(30.0f) == 7.5f);
        densityConfig.density.pairScanSuppression.cellSize = 4.0f;
        CHECK(densityConfig.density.pairScanSuppression.effectiveCellSize(30.0f) == 4.0f);
    }
    SUBCASE("Disabled pair scan suppression preserves the baseline graph") {
        BuildConfig disabledConfig = buildConfig;
        disabledConfig.density.pairScanSuppression.enabled = false;
        disabledConfig.density.pairScanSuppression.cellSizeRatio = 0.0f;
        const BuildResult result = builder.build(*navMesh, disabledConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(haveSameLinks(result.graph, baseline.graph));
        CHECK(result.stats.candidates.pairScanSuppressedCount == 0);
    }
    SUBCASE("Enabled pair scan suppression accounts for every eligible projection") {
        BuildConfig suppressionConfig = buildConfig;
        suppressionConfig.density.pairScanSuppression.enabled = true;
        suppressionConfig.density.pairScanSuppression.cellSize = 10.0f;
        const BuildResult result = builder.build(*navMesh, suppressionConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(
            result.stats.candidates.pairScanSuppressedCount +
            result.stats.candidates.closestPointQueryCount ==
            result.stats.candidates.pairScanCandidateCount);
        CHECK(result.stats.candidates.closestPointQueryCount <= baseline.stats.candidates.closestPointQueryCount);
    }
    SUBCASE("Stronger pair scan suppression does not increase projection queries") {
        BuildConfig weakConfig = buildConfig;
        weakConfig.density.pairScanSuppression.enabled = true;
        weakConfig.density.pairScanSuppression.cellSize = 1.0f;
        const BuildResult weakResult = builder.build(*navMesh, weakConfig);
        REQUIRE(static_cast<bool>(weakResult));

        BuildConfig strongConfig = weakConfig;
        strongConfig.density.pairScanSuppression.cellSize = 10.0f;
        const BuildResult strongResult = builder.build(*navMesh, strongConfig);
        REQUIRE(static_cast<bool>(strongResult));
        CHECK(strongResult.stats.candidates.closestPointQueryCount <= weakResult.stats.candidates.closestPointQueryCount);
    }
    SUBCASE("Pair scan suppression is deterministic") {
        BuildConfig suppressionConfig = buildConfig;
        suppressionConfig.density.pairScanSuppression.enabled = true;
        suppressionConfig.density.pairScanSuppression.cellSize = 10.0f;
        const BuildResult first = builder.build(*navMesh, suppressionConfig);
        const BuildResult second = builder.build(*navMesh, suppressionConfig);
        REQUIRE(static_cast<bool>(first));
        REQUIRE(static_cast<bool>(second));
        CHECK(haveSameLinks(first.graph, second.graph));
        CHECK(first.stats.candidates.pairScanSuppressedCount == second.stats.candidates.pairScanSuppressedCount);
    }
    SUBCASE("Rejects invalid pair scan suppression cell sizes") {
        BuildConfig invalidConfig(30.0f, 30.0f, 30.0f);
        invalidConfig.density.pairScanSuppression.enabled = true;
        invalidConfig.density.pairScanSuppression.cellSize = -0.01f;
        CHECK(builder.build(*navMesh, invalidConfig).status == BuildStatus::InvalidConfiguration);

        invalidConfig.density.pairScanSuppression.cellSize = 0.0f;
        invalidConfig.density.pairScanSuppression.cellSizeRatio = 0.0f;
        CHECK(builder.build(*navMesh, invalidConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Boundary representative reduction can be enabled independently") {
        BuildConfig representativeConfig = buildConfig;
        representativeConfig.boundaries.representativeReductionEnabled = true;
        representativeConfig.boundaries.representativeCellSize = 10.0f;
        const BuildResult result = builder.build(*navMesh, representativeConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.boundaries.representativeCount <= result.stats.boundaries.deduplicatedCount);
        CHECK(
            result.stats.boundaries.representativeTrimmedCount +
            result.stats.boundaries.representativeCount ==
            result.stats.boundaries.deduplicatedCount);
        CHECK(result.stats.queries.count == result.stats.boundaries.representativeCount);
    }
    SUBCASE("Boundary representative direction buckets are angular") {
        BuildConfig representativeConfig = buildConfig;
        representativeConfig.boundaries.representativeDirectionBuckets = 8;
        CHECK(representativeConfig.boundaries.representativeDirectionBucket({1.0f, 0.0f, 0.0f}) == 0);
        CHECK(representativeConfig.boundaries.representativeDirectionBucket({10.0f, 0.0f, 0.0f}) == 0);
        CHECK(representativeConfig.boundaries.representativeDirectionBucket({0.0f, 0.0f, 1.0f}) == 2);
        CHECK(representativeConfig.boundaries.representativeDirectionBucket({-1.0f, 0.0f, 0.0f}) == 4);
        CHECK(representativeConfig.boundaries.representativeDirectionBucket({0.0f, 0.0f, -1.0f}) == 6);
    }
    SUBCASE("Disabled boundary representative reduction preserves deduplicated boundaries") {
        BuildConfig representativeConfig = buildConfig;
        representativeConfig.boundaries.representativeReductionEnabled = false;
        representativeConfig.boundaries.representativeCellSizeRatio = 0.0f;
        const BuildResult result = builder.build(*navMesh, representativeConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.boundaries.representativeCount == result.stats.boundaries.deduplicatedCount);
        CHECK(result.stats.boundaries.representativeTrimmedCount == 0);
    }
    SUBCASE("Boundary representative ranking is customizable") {
        BuildConfig representativeConfig = buildConfig;
        representativeConfig.boundaries.representativeReductionEnabled = true;
        representativeConfig.boundaries.representativeCellSize = 10.0f;
        std::size_t rankCallCount = 0;
        representativeConfig.boundaries.representativeRanker =
            [&rankCallCount](
                const BoundaryRepresentativeCandidate& candidate,
                const IslandGraph& graph) {
                ++rankCallCount;
                CHECK(candidate.island < graph.islands().size());
                return candidate.midpoint.x;
            };
        const BuildResult result = builder.build(*navMesh, representativeConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(rankCallCount == result.stats.boundaries.deduplicatedCount);
    }
    SUBCASE("Rejects invalid boundary representative cell size ratio") {
        BuildConfig invalidRepresentativeConfig = buildConfig;
        invalidRepresentativeConfig.boundaries.representativeReductionEnabled = true;
        invalidRepresentativeConfig.boundaries.representativeCellSizeRatio = 0.0f;
        CHECK(builder.build(*navMesh, invalidRepresentativeConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Rejects invalid boundary representative direction bucket count") {
        BuildConfig invalidRepresentativeConfig = buildConfig;
        invalidRepresentativeConfig.boundaries.representativeReductionEnabled = true;
        invalidRepresentativeConfig.boundaries.representativeDirectionBuckets = 0;
        CHECK(builder.build(*navMesh, invalidRepresentativeConfig).status == BuildStatus::InvalidConfiguration);
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
        BuildConfig invalidDensityConfig(30.0f, 30.0f, 30.0f);
        invalidDensityConfig.density.localPruning.enableDistanceScaling = true;
        invalidDensityConfig.density.localPruning.distanceScale = -0.01f;
        CHECK(builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Rejects negative density distance scale ratio") {
        BuildConfig invalidDensityConfig(30.0f, 30.0f, 30.0f);
        invalidDensityConfig.density.localPruning.enableDistanceScaling = true;
        invalidDensityConfig.density.localPruning.distanceScaleRatio = -0.01f;
        CHECK(builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Rejects invalid candidate deduplication cell sizes") {
        BuildConfig invalidDensityConfig(30.0f, 30.0f, 30.0f);
        invalidDensityConfig.density.candidateDeduplication.enabled = true;
        invalidDensityConfig.density.candidateDeduplication.cellSize = -0.01f;
        CHECK(builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration);

        invalidDensityConfig.density.candidateDeduplication.cellSize = 0.0f;
        invalidDensityConfig.density.candidateDeduplication.farCellSizeRatio = 0.0f;
        CHECK(builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Rejects density caps below one") {
        BuildConfig invalidDensityConfig(30.0f, 30.0f, 30.0f);
        invalidDensityConfig.density.localPruning.enableDistanceScaling = true;
        invalidDensityConfig.density.localPruning.maxRadiusScale = 0.99f;
        CHECK(builder.build(*navMesh, invalidDensityConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Disabled global pruning preserves baseline") {
        BuildConfig disabledGlobalPruningConfig = buildConfig;
        disabledGlobalPruningConfig.density.globalPruning.enabled = false;
        disabledGlobalPruningConfig.density.globalPruning.nearRadiusRatio = 0.0f;
        disabledGlobalPruningConfig.density.globalPruning.farRadiusRatio = 0.0f;
        const BuildResult result = builder.build(*navMesh, disabledGlobalPruningConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount == baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Enabled global pruning does not increase accepted links") {
        BuildConfig globalPruningConfig = buildConfig;
        globalPruningConfig.density.globalPruning.enabled = true;
        globalPruningConfig.density.globalPruning.nearRadiusRatio = 2.0f;
        globalPruningConfig.density.globalPruning.farRadiusRatio = 2.0f;
        const BuildResult result = builder.build(*navMesh, globalPruningConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= baseline.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Stronger global pruning does not increase accepted links") {
        BuildConfig globalPruningConfig = buildConfig;
        globalPruningConfig.density.globalPruning.enabled = true;
        globalPruningConfig.density.globalPruning.nearRadiusRatio = 2.0f;
        globalPruningConfig.density.globalPruning.farRadiusRatio = 2.0f;
        const BuildResult globalPruningBuild = builder.build(*navMesh, globalPruningConfig);
        REQUIRE(static_cast<bool>(globalPruningBuild));

        BuildConfig strongerGlobalPruningConfig = buildConfig;
        strongerGlobalPruningConfig.density.globalPruning.enabled = true;
        strongerGlobalPruningConfig.density.globalPruning.nearRadiusRatio = 4.0f;
        strongerGlobalPruningConfig.density.globalPruning.farRadiusRatio = 4.0f;
        const BuildResult result = builder.build(*navMesh, strongerGlobalPruningConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= globalPruningBuild.stats.candidates.acceptedLinkCount);
    }
    SUBCASE("Global pruning radius responds to link distance") {
        BuildConfig densityConfig(30.0f, 30.0f, 30.0f);
        densityConfig.density.globalPruning.enabled = true;
        densityConfig.density.globalPruning.nearRadiusRatio = 0.5f;
        densityConfig.density.globalPruning.farRadiusRatio = 1.0f;
        CHECK(densityConfig.density.globalPruning.effectiveRadius(0.0f, 30.0f) == 15.0f);
        CHECK(densityConfig.density.globalPruning.effectiveRadius(15.0f, 30.0f) == 22.5f);
        CHECK(densityConfig.density.globalPruning.effectiveRadius(60.0f, 30.0f) == 30.0f);
    }
    SUBCASE("Rejects negative global prune radius ratios") {
        BuildConfig invalidGlobalPruningConfig(30.0f, 30.0f, 30.0f);
        invalidGlobalPruningConfig.density.globalPruning.enabled = true;
        invalidGlobalPruningConfig.density.globalPruning.nearRadiusRatio = -0.01f;
        CHECK(builder.build(*navMesh, invalidGlobalPruningConfig).status == BuildStatus::InvalidConfiguration);

        invalidGlobalPruningConfig.density.globalPruning.nearRadiusRatio = 0.5f;
        invalidGlobalPruningConfig.density.globalPruning.farRadiusRatio = -0.01f;
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
        BuildConfig invalidSpannerConfig(30.0f, 30.0f, 30.0f);
        invalidSpannerConfig.density.spannerPruning.enabled = true;
        invalidSpannerConfig.density.spannerPruning.pathRatio = 0.99f;
        CHECK(builder.build(*navMesh, invalidSpannerConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Rejects negative spanner vertical weight") {
        BuildConfig invalidSpannerConfig(30.0f, 30.0f, 30.0f);
        invalidSpannerConfig.density.spannerPruning.enabled = true;
        invalidSpannerConfig.density.spannerPruning.verticalWeight = -0.01f;
        CHECK(builder.build(*navMesh, invalidSpannerConfig).status == BuildStatus::InvalidConfiguration);
    }
}

TEST_CASE("Builder polygon filtering and ranking") {
    const auto navMesh = buildDisconnectedNavMesh();
    const IslandGraphBuilder builder;

    SUBCASE("Detour include flags restrict island extraction") {
        BuildConfig config(3.0f, 2.0f, 4.0f);
        config.query.includeFlags = 2;
        const BuildResult result = builder.build(*navMesh, config);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.graph.empty());
    }
    SUBCASE("Polygon predicate restricts island extraction") {
        BuildConfig config(3.0f, 2.0f, 4.0f);
        config.polygonFilter = [](dtPolyRef, const dtMeshTile&, const dtPoly&) {
            return false;
        };
        const BuildResult result = builder.build(*navMesh, config);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.graph.empty());
    }
    SUBCASE("Custom link ranker participates in candidate selection") {
        BuildConfig config(3.0f, 2.0f, 4.0f);
        int rankCalls = 0;
        config.linkRanker = [&rankCalls](const Link& link, const IslandGraph&) {
            ++rankCalls;
            return link.horizontalDistance;
        };
        const BuildResult result = builder.build(*navMesh, config);
        REQUIRE(static_cast<bool>(result));
        CHECK(rankCalls > 0);
    }
}

TEST_CASE("Builder pruning diagnostics") {
    const auto navMesh = buildDisconnectedNavMesh();
    BuildConfig buildConfig(30.0f, 30.0f, 30.0f);
    buildConfig.gapDiscovery.maxHorizontalGap = 3.0f;
    buildConfig.gapDiscovery.maxVerticalGapUp = 2.0f;
    buildConfig.gapDiscovery.maxVerticalGapDown = 4.0f;
    buildConfig.boundaries.deduplicationCellSize = 0.5f;
    buildConfig.density.localPruning.baseRadius = 0.5f;
    const IslandGraphBuilder builder;
    const BuildResult baseline = builder.build(*navMesh, buildConfig);
    REQUIRE(static_cast<bool>(baseline));

    SUBCASE("All reject counters are zero when pruning is disabled") {
        BuildConfig disabledConfig = buildConfig;
        disabledConfig.density.globalPruning.enabled = false;
        disabledConfig.density.spannerPruning.enabled = false;
        disabledConfig.density.localPruning.enabled = false;
        const BuildResult result = builder.build(*navMesh, disabledConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.globalPruningRejectCount == 0);
        CHECK(result.stats.candidates.spannerPruningRejectCount == 0);
        CHECK(result.stats.candidates.localPruningRejectCount == 0);
    }
    SUBCASE("Pruning reject counts and accepted count sum to deduplicated count") {
        BuildConfig allPruningConfig = buildConfig;
        allPruningConfig.density.globalPruning.enabled = true;
        allPruningConfig.density.globalPruning.nearRadiusRatio = 2.0f;
        allPruningConfig.density.globalPruning.farRadiusRatio = 2.0f;
        allPruningConfig.density.spannerPruning.enabled = true;
        allPruningConfig.density.spannerPruning.pathRatio = 2.0f;
        const BuildResult result = builder.build(*navMesh, allPruningConfig);
        REQUIRE(static_cast<bool>(result));
        const std::size_t totalProcessed =
            result.stats.candidates.globalPruningRejectCount +
            result.stats.candidates.spannerPruningRejectCount +
            result.stats.candidates.localPruningRejectCount +
            result.stats.candidates.acceptedLinkCount;
        CHECK(totalProcessed == result.stats.candidates.deduplicatedCount);
    }
    SUBCASE("Stronger pruning increases reject counts") {
        BuildConfig weakConfig = buildConfig;
        weakConfig.density.globalPruning.enabled = true;
        weakConfig.density.globalPruning.nearRadiusRatio = 1.0f;
        weakConfig.density.globalPruning.farRadiusRatio = 1.0f;
        weakConfig.density.localPruning.enabled = true;
        weakConfig.density.localPruning.baseRadius = 2.0f;
        const BuildResult weakResult = builder.build(*navMesh, weakConfig);
        REQUIRE(static_cast<bool>(weakResult));

        BuildConfig strongConfig = buildConfig;
        strongConfig.density.globalPruning.enabled = true;
        strongConfig.density.globalPruning.nearRadiusRatio = 4.0f;
        strongConfig.density.globalPruning.farRadiusRatio = 4.0f;
        strongConfig.density.localPruning.enabled = true;
        strongConfig.density.localPruning.baseRadius = 10.0f;
        const BuildResult strongResult = builder.build(*navMesh, strongConfig);
        REQUIRE(static_cast<bool>(strongResult));
        CHECK(strongResult.stats.candidates.globalPruningRejectCount >= weakResult.stats.candidates.globalPruningRejectCount);
        CHECK(strongResult.stats.candidates.localPruningRejectCount >= weakResult.stats.candidates.localPruningRejectCount);
    }
    SUBCASE("Graph health metrics are populated") {
        CHECK(baseline.stats.maxOutgoingLinksOnIsland > 0);
        CHECK(baseline.stats.maxIncomingLinksOnIsland > 0);
        CHECK(baseline.stats.islandsWithOutgoingLinks > 0);
        CHECK(baseline.stats.islandsWithIncomingLinks > 0);
        CHECK(baseline.stats.connectedComponentCount > 0);
        CHECK(baseline.stats.largestConnectedComponentIslandCount > 0);
        CHECK(baseline.stats.averageLinkLength > 0.0);
    }
}

TEST_CASE("Builder mass-aware tuning") {
    const auto variedMassNavMesh = buildVariedMassNavMesh();
    const IslandGraphBuilder builder;

    SUBCASE("Baseline preserves zero scores") {
        const BuildResult geometricMassBuild =
            builder.build(*variedMassNavMesh, BuildConfig(30.0f, 30.0f, 30.0f));
        REQUIRE(static_cast<bool>(geometricMassBuild));
        CHECK(geometricMassBuild.graph.islands()[0].massScore == 0.0f);
        CHECK(geometricMassBuild.graph.islands()[1].massScore == 0.0f);
    }
    SUBCASE("Target preference and radius scale interpolate continuously") {
        BuildConfig massAwareConfig(30.0f, 30.0f, 30.0f);
        massAwareConfig.massAware.enabled = true;
        massAwareConfig.massAware.targetPreference = 2.0f;
        massAwareConfig.massAware.lowMassPruneRadiusScale = 1.5f;
        massAwareConfig.massAware.highMassPruneRadiusScale = 0.75f;
        CHECK(massAwareConfig.massAware.targetPreferenceFor(0.5f, 30.0f) == 1.0f);
        CHECK(massAwareConfig.massAware.pruneRadiusScaleFor(0.5f) == 1.125f);
    }
    SUBCASE("Target preference supports a horizontal-gap ratio") {
        BuildConfig massAwareConfig(30.0f, 30.0f, 30.0f);
        massAwareConfig.massAware.targetPreferenceRatio = 0.5f;
        CHECK(massAwareConfig.massAware.targetPreferenceFor(0.5f, 30.0f) == 7.5f);
    }
    SUBCASE("Mass-aware scores are normalized and ordered") {
        BuildConfig massAwareConfig(30.0f, 30.0f, 30.0f);
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
        BuildConfig massAwareConfig(30.0f, 30.0f, 30.0f);
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
    SUBCASE("Mass scores are independent of coordinate scale") {
        const auto scaledNavMesh = buildVariedMassNavMesh(10);
        BuildConfig massAwareConfig(30.0f, 30.0f, 30.0f);
        massAwareConfig.massAware.enabled = true;
        const BuildResult baseline = builder.build(*variedMassNavMesh, massAwareConfig);
        const BuildResult scaled = builder.build(*scaledNavMesh, massAwareConfig);
        REQUIRE(static_cast<bool>(baseline));
        REQUIRE(static_cast<bool>(scaled));
        REQUIRE(baseline.graph.islands().size() == scaled.graph.islands().size());
        for (std::size_t index = 0; index < baseline.graph.islands().size(); ++index) {
            CHECK(baseline.graph.islands()[index].massScore == scaled.graph.islands()[index].massScore);
        }
    }
    SUBCASE("Rejects invalid mass normalization percentile") {
        const auto navMesh = buildDisconnectedNavMesh();
        BuildConfig invalidMassAwareConfig(30.0f, 30.0f, 30.0f);
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
        BuildConfig massAwareConfig(30.0f, 30.0f, 30.0f);
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
    SUBCASE("Read limits are configurable") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        serialized.seekg(0);
        DeserializationLimits limits;
        limits.maxIslandCount = 2;
        CHECK(IslandGraphSerializer::read(serialized, limits).status == SerializationStatus::MalformedData);
    }
}
