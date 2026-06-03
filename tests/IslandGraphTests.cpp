#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <detour_island_graph/IslandGraph.h>
#include <detour_island_graph/IslandGraphBuilder.h>
#include <detour_island_graph/IslandGraphPathfinder.h>
#include <detour_island_graph/IslandGraphSerializer.h>

#include "../src/IslandGraphDiscoveryInternal.h"

#include <DetourAlloc.h>
#include <DetourNavMeshBuilder.h>

#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

namespace {
using detour_island_graph::detail::discovery::Boundary;
using detour_island_graph::detail::discovery::pruneCandidates;
using detour_island_graph::detail::discovery::selectBoundaryRepresentatives;

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
    for (std::uint32_t edgeIndex : island->edgeIndices) {
        if (edgeIndex >= graph.edges().size()) {
            continue;
        }
        const std::optional<detour_island_graph::Link> traversal =
            detour_island_graph::makeTraversalLink(graph.edges()[edgeIndex], from);
        if (traversal.has_value() && traversal->toIsland == to) {
            return true;
        }
    }
    return false;
}

detour_island_graph::Edge makeEdge(const detour_island_graph::Link& link, bool reverse = false) {
    detour_island_graph::Edge edge;
    edge.islandA = link.fromIsland;
    edge.islandB = link.toIsland;
    edge.pointA = link.start;
    edge.pointB = link.end;
    edge.horizontalDistance = link.horizontalDistance;
    edge.verticalDeltaAB = link.verticalDistance;
    edge.traversableAB = true;
    edge.traversableBA = reverse;
    return edge;
}

detour_island_graph::IslandGraph makeGraph(
    std::vector<detour_island_graph::Island> islands,
    std::vector<detour_island_graph::Edge> edges = {}) {
    return detour_island_graph::IslandGraph(std::move(islands), std::move(edges));
}

detour_island_graph::Island makeIsland(
    std::uint32_t id,
    std::vector<dtPolyRef> polygons = {}) {
    detour_island_graph::Island island;
    island.id = id;
    island.polygons = std::move(polygons);
    return island;
}

detour_island_graph::BuildConfig disconnectedBuildConfig() {
    detour_island_graph::BuildConfig config(3.0f, 2.0f, 4.0f);
    config.boundaries.deduplicationCellSize = 0.5f;
    config.density.localPruning.baseRadius = 0.5f;
    return config;
}

} // namespace

using namespace detour_island_graph;

TEST_CASE("IslandGraph indexes valid islands and polygons") {
    IslandGraph emptyGraph;
    CHECK(emptyGraph.empty());
    CHECK(emptyGraph.findIsland(0) == nullptr);
    CHECK(!emptyGraph.findIslandForPolygon(42).has_value());

    IslandGraph graph({makeIsland(0, {42, 43})});
    CHECK_FALSE(graph.empty());
    REQUIRE(graph.findIsland(0) != nullptr);
    CHECK(graph.findIsland(0)->polygons.size() == 2);
    CHECK(graph.findIsland(1) == nullptr);
    CHECK(graph.findIslandForPolygon(42) == 0);
    CHECK(!graph.findIslandForPolygon(99).has_value());

    IslandGraph inconsistentGraph({makeIsland(7, {42})});
    CHECK(inconsistentGraph.findIsland(0) == nullptr);
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
    CHECK(sparse.density.shortGapRecovery.enabled);
    CHECK_FALSE(sparse.density.globalPruning.enabled);
    CHECK(sparse.density.spannerPruning.enabled);

    const BuildConfig unpruned =
        BuildConfig::forProfile(BuildProfile::Unpruned, 30.0f, 30.0f, 30.0f);
    CHECK_FALSE(unpruned.boundaries.deduplicationEnabled);
    CHECK_FALSE(unpruned.density.candidateDeduplication.enabled);
    CHECK_FALSE(unpruned.density.localPruning.enabled);
}

TEST_CASE("Pathfinder routes, filters, and reports edge cases") {
    const Link direct{0, 2, {0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}, 100.0f, 0.0f};
    const Link firstHop{0, 1, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link secondHop{1, 2, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    IslandGraph routeGraph = makeGraph(
        {makeIsland(0), makeIsland(1), makeIsland(2)},
        {makeEdge(direct), makeEdge(firstHop), makeEdge(secondHop)});
    const IslandGraphPathfinder pathfinder;

    SUBCASE("Uses the cheaper route") {
        const PathResult path = pathfinder.findPath(
            routeGraph, 0, 2, {0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f});
        REQUIRE(path.status == PathStatus::Success);
        CHECK(path.links.size() == 2);
        CHECK(path.totalCost == 3.0f);
    }
    SUBCASE("A filter can reject an otherwise preferred hop") {
        const PathResult path = pathfinder.findPath(
            routeGraph,
            0,
            2,
            {},
            {},
            PathOptions{{}, [](const Link& link, const LinkCostContext&) { return link.toIsland != 1; }, {}});
        REQUIRE(path.status == PathStatus::Success);
        CHECK(path.links.size() == 1);
    }
    SUBCASE("Reports same and invalid islands") {
        CHECK(pathfinder.findPath(routeGraph, 0, 0, {}, {}).status == PathStatus::SameIsland);
        CHECK(pathfinder.findPath(routeGraph, 0, 9, {}, {}).status == PathStatus::InvalidIsland);
    }
    SUBCASE("Ignores malformed links") {
        const Link wrongSource{7, 1, {}, {}, 1.0f, 0.0f};
        const Link invalidTarget{0, 9, {}, {}, 1.0f, 0.0f};
        IslandGraph malformedGraph = makeGraph(
            {makeIsland(0), makeIsland(1)},
            {makeEdge(wrongSource), makeEdge(invalidTarget)});
        CHECK(pathfinder.findPath(malformedGraph, 0, 1, {}, {}).status == PathStatus::NoPath);
    }
}

TEST_CASE("Pathfinder custom costs use an admissible search order") {
    const Link expensiveHop{0, 2, {}, {}, 10.0f, 0.0f};
    const Link cheapHop{0, 1, {}, {100.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link bridge{1, 2, {100.0f, 0.0f, 0.0f}, {}, 1.0f, 0.0f};
    const Link goal{2, 3, {}, {}, 1.0f, 0.0f};
    IslandGraph routeGraph = makeGraph(
        {makeIsland(0), makeIsland(1), makeIsland(2), makeIsland(3)},
        {makeEdge(expensiveHop), makeEdge(cheapHop), makeEdge(bridge), makeEdge(goal)});
    const IslandGraphPathfinder pathfinder;
    int heuristicCalls = 0;
    const PathResult path = pathfinder.findPath(
        routeGraph,
        0,
        3,
        {},
        {},
        PathOptions{
            [](const Link& link, const LinkCostContext&) { return link.horizontalDistance; },
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

TEST_CASE("Pathfinder callbacks receive route context") {
    const Link firstHop{0, 1, {}, {}, 1.0f, 0.0f};
    const Link finalHop{1, 2, {}, {}, 1.0f, 0.0f};
    IslandGraph routeGraph = makeGraph(
        {makeIsland(0), makeIsland(1), makeIsland(2)},
        {makeEdge(firstHop), makeEdge(finalHop)});
    const IslandGraphPathfinder pathfinder;
    bool sawIntermediateHop = false;
    bool sawFinalHop = false;
    bool filterSawContext = false;
    const PathResult path = pathfinder.findPath(
        routeGraph,
        0,
        2,
        {},
        {},
        PathOptions{
            [&](const Link& link, const LinkCostContext& context) {
                CHECK(&context.graph == &routeGraph);
                CHECK(context.startIsland == 0);
                CHECK(context.endIsland == 2);
                sawIntermediateHop |= link.toIsland != context.endIsland;
                sawFinalHop |= link.toIsland == context.endIsland;
                return link.horizontalDistance;
            },
            [&](const Link&, const LinkCostContext& context) {
                filterSawContext = &context.graph == &routeGraph &&
                    context.startIsland == 0 &&
                    context.endIsland == 2;
                return true;
            },
            {}});
    REQUIRE(path.status == PathStatus::Success);
    CHECK(sawIntermediateHop);
    CHECK(sawFinalHop);
    CHECK(filterSawContext);
}

TEST_CASE("Builder with disconnected navmesh") {
    const auto navMesh = buildDisconnectedNavMesh();
    const BuildConfig buildConfig = disconnectedBuildConfig();
    const IslandGraphBuilder builder;
    const BuildResult buildResult = builder.build(*navMesh, buildConfig);
    REQUIRE(static_cast<bool>(buildResult));

    CHECK(buildResult.graph.islands().size() == 3);
    CHECK(hasLink(buildResult.graph, 0, 1));
    CHECK(hasLink(buildResult.graph, 1, 0));
    CHECK_FALSE(hasLink(buildResult.graph, 1, 2));
    CHECK(hasLink(buildResult.graph, 2, 1));
    CHECK(buildResult.stats.maxOutgoingLinksOnIsland > 0);
    CHECK(buildResult.stats.maxIncomingLinksOnIsland > 0);
    CHECK(buildResult.stats.connectedComponentCount > 0);
    CHECK(buildResult.stats.largestConnectedComponentIslandCount > 0);
    CHECK(buildResult.stats.averageLinkLength > 0.0);
    for (const Island& island : buildResult.graph.islands()) {
        bool sawTraversableEdge = false;
        for (std::uint32_t edgeIndex : island.edgeIndices) {
            REQUIRE(edgeIndex < buildResult.graph.edges().size());
            const auto traversal = makeTraversalLink(buildResult.graph.edges()[edgeIndex], island.id);
            if (!traversal.has_value()) {
                continue;
            }
            sawTraversableEdge = true;
            CHECK(traversal->fromIsland != traversal->toIsland);
        }
        CHECK((sawTraversableEdge || island.id == 1) == true);
    }

    SUBCASE("Outbound island filtering preserves islands and incoming links") {
        BuildConfig filteredConfig = buildConfig;
        filteredConfig.outboundIslandFilter = [](const Island& island, const IslandGraph&) {
            return island.id != 0;
        };
        const BuildResult filtered = builder.build(*navMesh, filteredConfig);
        REQUIRE(static_cast<bool>(filtered));
        CHECK(filtered.graph.islands().size() == buildResult.graph.islands().size());
        CHECK_FALSE(hasLink(filtered.graph, 0, 1));
        CHECK(hasLink(filtered.graph, 1, 0));
    }
}

TEST_CASE("Builder cancellation is cooperative and discards partial graph") {
    const auto navMesh = buildDisconnectedNavMesh();
    const IslandGraphBuilder builder;
    const BuildConfig config(30.0f, 30.0f, 30.0f);

    SUBCASE("Cancellation before work returns a distinct result") {
        const BuildResult result = builder.build(*navMesh, config, BuildOptions{[] { return true; }});
        CHECK(result.status == BuildStatus::Cancelled);
        CHECK(result.message == "Build cancelled.");
        CHECK(result.graph.empty());
    }
    SUBCASE("Cancellation during work does not expose partial topology") {
        int checks = 0;
        const BuildResult result = builder.build(
            *navMesh,
            config,
            BuildOptions{[&] {
                ++checks;
                return checks >= 4;
            }});
        CHECK(result.status == BuildStatus::Cancelled);
        CHECK(checks >= 4);
        CHECK(result.graph.empty());
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

TEST_CASE("Builder rejects non-finite navmesh boundary coordinates") {
    const auto navMesh = buildDisconnectedNavMesh();
    const dtMeshTile* tile = static_cast<const dtNavMesh&>(*navMesh).getTile(0);
    REQUIRE(tile != nullptr);
    tile->verts[0] = (std::numeric_limits<float>::quiet_NaN)();

    const BuildResult result =
        IslandGraphBuilder{}.build(*navMesh, disconnectedBuildConfig());
    CHECK(result.status == BuildStatus::InvalidNavMesh);
    CHECK(result.message == "Navmesh boundary contains non-finite coordinates.");
}

TEST_CASE("Builder density tuning") {
    const auto navMesh = buildDisconnectedNavMesh();
    const BuildConfig buildConfig = disconnectedBuildConfig();
    const IslandGraphBuilder builder;
    const BuildResult baseline = builder.build(*navMesh, buildConfig);
    REQUIRE(static_cast<bool>(baseline));

    SUBCASE("Local pruning can be disabled independently") {
        BuildConfig unprunedConfig = buildConfig;
        unprunedConfig.density.localPruning.enabled = false;
        unprunedConfig.density.localPruning.baseRadius = 0.0f;
        const BuildResult result = builder.build(*navMesh, unprunedConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount >= baseline.stats.candidates.acceptedLinkCount);
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
    SUBCASE("Enabled pair scan suppression reduces projection work") {
        BuildConfig suppressionConfig = buildConfig;
        suppressionConfig.density.pairScanSuppression.enabled = true;
        suppressionConfig.density.pairScanSuppression.cellSize = 10.0f;
        const BuildResult result = builder.build(*navMesh, suppressionConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.pairScanSuppressedCount > 0);
        CHECK(result.stats.candidates.closestPointQueryCount <= baseline.stats.candidates.closestPointQueryCount);
    }
    SUBCASE("Short-gap recovery rescans boundaries after pair suppression") {
        BuildConfig recoveryConfig = buildConfig;
        recoveryConfig.density.pairScanSuppression.enabled = true;
        recoveryConfig.density.pairScanSuppression.cellSize = 10.0f;
        recoveryConfig.density.shortGapRecovery.enabled = true;
        recoveryConfig.density.shortGapRecovery.maxHorizontalGapRatio = 1.0f;
        const BuildResult result = builder.build(*navMesh, recoveryConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.shortGapRecoveryQueryCount > 0);
        CHECK(result.stats.candidates.shortGapRecoveredCount > 0);
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
    SUBCASE("Rejects invalid short-gap recovery ratios") {
        BuildConfig invalidConfig(30.0f, 30.0f, 30.0f);
        invalidConfig.density.shortGapRecovery.enabled = true;
        invalidConfig.density.shortGapRecovery.maxHorizontalGapRatio = 0.0f;
        CHECK(builder.build(*navMesh, invalidConfig).status == BuildStatus::InvalidConfiguration);
    }
    SUBCASE("Short-gap recovery cannot exceed the configured agent gap") {
        BuildConfig recoveryConfig(30.0f, 30.0f, 30.0f);
        recoveryConfig.density.shortGapRecovery.maxHorizontalGap = 60.0f;
        CHECK(recoveryConfig.density.shortGapRecovery.effectiveMaxHorizontalGap(30.0f) == 30.0f);
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
        CHECK(rankCallCount > 0);
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
    SUBCASE("Rejects negative density distance scale") {
        BuildConfig invalidDensityConfig(30.0f, 30.0f, 30.0f);
        invalidDensityConfig.density.localPruning.enableDistanceScaling = true;
        invalidDensityConfig.density.localPruning.distanceScale = -0.01f;
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
    SUBCASE("Enabled global pruning does not increase accepted links") {
        BuildConfig globalPruningConfig = buildConfig;
        globalPruningConfig.density.globalPruning.enabled = true;
        globalPruningConfig.density.globalPruning.nearRadiusRatio = 2.0f;
        globalPruningConfig.density.globalPruning.farRadiusRatio = 2.0f;
        const BuildResult result = builder.build(*navMesh, globalPruningConfig);
        REQUIRE(static_cast<bool>(result));
        CHECK(result.stats.candidates.acceptedLinkCount <= baseline.stats.candidates.acceptedLinkCount);
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

TEST_CASE("Boundary representative reduction preserves clearly separated vertical layers") {
    IslandGraph graph({makeIsland(0)});
    std::vector<Boundary> boundaries{
        Boundary{0, 1, {0.0f, 0.0f, 0.0f}, {4.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}},
        Boundary{0, 2, {0.0f, 6.0f, 0.0f}, {4.0f, 6.0f, 0.0f}, {2.0f, 6.0f, 0.0f}},
        Boundary{0, 3, {0.0f, 0.0f, 6.0f}, {4.0f, 0.0f, 6.0f}, {2.0f, 0.0f, 6.0f}}};
    BuildConfig config(8.0f, 8.0f, 8.0f);
    config.boundaries.representativeReductionEnabled = true;
    config.boundaries.representativeCellSize = 4.0f;
    BuildStats stats;
    std::vector<Boundary> representatives;

    const BuildStatus status = selectBoundaryRepresentatives(
        boundaries,
        graph,
        config,
        BuildOptions{},
        stats,
        representatives);

    REQUIRE(status == BuildStatus::Success);
    CHECK(representatives.size() == 3);
    CHECK(stats.boundaries.representativeTrimmedCount == 0);
}

TEST_CASE("Local pruning collapses nearby source-side corridors across fragmented targets") {
    IslandGraph graph({makeIsland(0), makeIsland(1)});
    BuildConfig config(8.0f, 8.0f, 8.0f);
    config.density.localPruning.enabled = true;
    config.density.localPruning.baseRadius = 3.0f;
    config.density.spannerPruning.enabled = false;
    config.density.globalPruning.enabled = false;
    config.massAware.enabled = true;
    graph = IslandGraph({makeIsland(0), makeIsland(1), makeIsland(2)});
    detour_island_graph::detail::IslandGraphAccess::islands(graph)[0].massScore = 1.0f;
    detour_island_graph::detail::IslandGraphAccess::islands(graph)[1].massScore = 1.0f;
    detour_island_graph::detail::IslandGraphAccess::islands(graph)[2].massScore = 0.1f;
    BuildStats stats;
    std::vector<Link> candidates{
        Link{0, 1, {0.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f}, 5.0f, 0.0f},
        Link{0, 2, {0.0f, 5.0f, 0.0f}, {5.0f, 5.0f, 0.0f}, 5.0f, 0.0f},
        Link{0, 2, {0.0f, 6.0f, 0.0f}, {5.0f, 6.0f, 0.0f}, 5.0f, 0.0f},
        Link{0, 1, {0.0f, 0.0f, 5.0f}, {5.0f, 0.0f, 5.0f}, 5.0f, 0.0f}};

    const BuildStatus status = pruneCandidates(graph, config, BuildOptions{}, stats, candidates);

    REQUIRE(status == BuildStatus::Success);
    REQUIRE(graph.findIsland(0) != nullptr);
    CHECK(graph.findIsland(0)->edgeIndices.size() == 2);
    CHECK(stats.candidates.localPruningRejectCount == 2);
    REQUIRE_FALSE(graph.findIsland(0)->edgeIndices.empty());
    const auto firstTraversal = makeTraversalLink(
        graph.edges()[graph.findIsland(0)->edgeIndices[0]],
        0);
    REQUIRE(firstTraversal.has_value());
    CHECK(firstTraversal->toIsland == 1);
    CHECK(hasLink(graph, 0, 2));
}

TEST_CASE("Local pruning aggressively collapses same-pair horizontal variants on one layer") {
    IslandGraph graph({makeIsland(0), makeIsland(1)});
    BuildConfig config(30.0f, 30.0f, 30.0f);
    config.density.localPruning.enabled = true;
    config.density.localPruning.baseRadius = 1.0f;
    config.density.spannerPruning.enabled = false;
    config.density.globalPruning.enabled = false;
    BuildStats stats;
    std::vector<Link> candidates{
        Link{0, 1, {0.0f, 10.0f, 0.0f}, {5.0f, 12.0f, 0.0f}, 5.0f, 2.0f},
        Link{0, 1, {20.0f, 10.2f, 0.0f}, {30.0f, 12.2f, 0.0f}, 10.0f, 2.0f}};

    const BuildStatus status = pruneCandidates(graph, config, BuildOptions{}, stats, candidates);

    REQUIRE(status == BuildStatus::Success);
    CHECK(graph.edges().size() == 1);
    CHECK(stats.candidates.localPruningRejectCount == 1);
    CHECK(hasLink(graph, 0, 1));
}

TEST_CASE("Local pruning preserves same-pair links on distinct vertical layers") {
    IslandGraph graph({makeIsland(0), makeIsland(1)});
    BuildConfig config(30.0f, 30.0f, 30.0f);
    config.density.localPruning.enabled = true;
    config.density.localPruning.baseRadius = 1.0f;
    config.density.spannerPruning.enabled = false;
    config.density.globalPruning.enabled = false;
    BuildStats stats;
    std::vector<Link> candidates{
        Link{0, 1, {0.0f, 10.0f, 0.0f}, {5.0f, 12.0f, 0.0f}, 5.0f, 2.0f},
        Link{0, 1, {20.0f, 18.0f, 0.0f}, {30.0f, 20.0f, 0.0f}, 10.0f, 2.0f}};

    const BuildStatus status = pruneCandidates(graph, config, BuildOptions{}, stats, candidates);

    REQUIRE(status == BuildStatus::Success);
    CHECK(graph.edges().size() == 2);
    CHECK(stats.candidates.localPruningRejectCount == 0);
}

TEST_CASE("Local pruning collapses same-pair links with matching vertical-distance bands") {
    IslandGraph graph({makeIsland(0), makeIsland(1)});
    BuildConfig config(30.0f, 30.0f, 30.0f);
    config.density.localPruning.enabled = true;
    config.density.localPruning.baseRadius = 1.0f;
    config.density.spannerPruning.enabled = false;
    config.density.globalPruning.enabled = false;
    BuildStats stats;
    std::vector<Link> candidates{
        Link{0, 1, {0.0f, 480.33f, 0.0f}, {5.0f, 486.08f, 0.0f}, 5.0f, 5.75f},
        Link{0, 1, {20.0f, 480.98f, 0.0f}, {30.0f, 488.48f, 0.0f}, 10.0f, 7.50f}};

    const BuildStatus status = pruneCandidates(graph, config, BuildOptions{}, stats, candidates);

    REQUIRE(status == BuildStatus::Success);
    CHECK(graph.edges().size() == 1);
    CHECK(stats.candidates.localPruningRejectCount == 1);
}

TEST_CASE("Symmetric spanner sees reverse traversal without duplicating stored edges") {
    IslandGraph graph({makeIsland(0), makeIsland(1), makeIsland(2)});
    BuildConfig config(30.0f, 30.0f, 30.0f);
    config.density.localPruning.enabled = false;
    config.density.globalPruning.enabled = false;
    config.density.spannerPruning.enabled = true;
    config.density.spannerPruning.pathRatio = 2.0f;
    BuildStats stats;
    std::vector<Link> candidates{
        Link{1, 0, {5.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 5.0f, 0.0f},
        Link{1, 2, {5.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f}, 5.0f, 0.0f},
        Link{0, 2, {0.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f}, 10.0f, 0.0f}};

    const BuildStatus status = pruneCandidates(graph, config, BuildOptions{}, stats, candidates);

    REQUIRE(status == BuildStatus::Success);
    CHECK(stats.candidates.spannerPruningRejectCount == 1);
    CHECK(graph.edges().size() == 2);
    CHECK(hasLink(graph, 0, 1));
    CHECK(hasLink(graph, 1, 0));
    CHECK(hasLink(graph, 1, 2));
    CHECK(hasLink(graph, 2, 1));
    CHECK_FALSE(hasLink(graph, 0, 2));
}

TEST_CASE("Symmetric local pruning collapses opposite-direction corridor samples") {
    IslandGraph graph({makeIsland(0), makeIsland(1)});
    BuildConfig config(30.0f, 30.0f, 30.0f);
    config.density.localPruning.enabled = true;
    config.density.localPruning.baseRadius = 1.0f;
    config.density.spannerPruning.enabled = false;
    config.density.globalPruning.enabled = false;
    BuildStats stats;
    std::vector<Link> candidates{
        Link{0, 1, {0.0f, 10.0f, 0.0f}, {5.0f, 12.0f, 0.0f}, 5.0f, 2.0f},
        Link{1, 0, {5.0f, 12.1f, 0.0f}, {0.0f, 10.1f, 0.0f}, 5.0f, -2.0f}};

    const BuildStatus status = pruneCandidates(graph, config, BuildOptions{}, stats, candidates);

    REQUIRE(status == BuildStatus::Success);
    CHECK(graph.edges().size() == 1);
    CHECK(stats.candidates.localPruningRejectCount == 1);
    CHECK(hasLink(graph, 0, 1));
    CHECK(hasLink(graph, 1, 0));
}

TEST_CASE("Builder stores one corridor and pathfinder traverses it both ways") {
    const auto navMesh = buildDisconnectedNavMesh();
    BuildConfig config(30.0f, 30.0f, 30.0f);
    config.boundaries.deduplicationCellSize = 0.5f;
    config.density.localPruning.baseRadius = 0.5f;

    const BuildResult result = IslandGraphBuilder{}.build(*navMesh, config);

    REQUIRE(static_cast<bool>(result));
    CHECK(hasLink(result.graph, 0, 1));
    CHECK(hasLink(result.graph, 1, 0));
    CHECK(hasLink(result.graph, 1, 2));
    CHECK(hasLink(result.graph, 2, 1));
    CHECK(result.graph.edges().size() == result.stats.candidates.acceptedLinkCount);
}

TEST_CASE("Graph health components follow traversal direction") {
    const Link oneWay{0, 1, {}, {1.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link twoWay{1, 2, {1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    IslandGraph graph = makeGraph(
        {makeIsland(0), makeIsland(1), makeIsland(2)},
        {makeEdge(oneWay), makeEdge(twoWay, true)});
    BuildStats stats;

    const BuildStatus status =
        detour_island_graph::detail::calculateGraphHealthStats(graph, BuildOptions{}, stats);

    REQUIRE(status == BuildStatus::Success);
    CHECK(stats.connectedComponentCount == 2);
    CHECK(stats.largestConnectedComponentIslandCount == 2);
    CHECK(stats.islandsWithOutgoingLinks == 3);
    CHECK(stats.islandsWithIncomingLinks == 2);
}

TEST_CASE("Symmetric candidate dedup collapses reverse duplicates into one edge") {
    IslandGraph graph({makeIsland(0), makeIsland(1)});
    BuildConfig config(30.0f, 30.0f, 30.0f);
    config.density.localPruning.enabled = false;
    config.density.spannerPruning.enabled = false;
    config.density.globalPruning.enabled = false;
    config.density.candidateDeduplication.enabled = true;
    config.density.candidateDeduplication.cellSize = 2.0f;
    BuildStats stats;
    std::vector<Link> candidates{
        Link{0, 1, {0.0f, 10.0f, 0.0f}, {5.0f, 12.0f, 0.0f}, 5.0f, 2.0f},
        Link{1, 0, {5.0f, 12.0f, 0.0f}, {0.0f, 10.0f, 0.0f}, 5.0f, -2.0f}};

    const BuildStatus status = pruneCandidates(graph, config, BuildOptions{}, stats, candidates);

    REQUIRE(status == BuildStatus::Success);
    CHECK(graph.edges().size() == 1);
    CHECK(hasLink(graph, 0, 1));
    CHECK(hasLink(graph, 1, 0));
}

TEST_CASE("Serializer") {
    const Link direct{0, 2, {0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}, 100.0f, 0.0f};
    const Link firstHop{0, 1, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    const Link secondHop{1, 2, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, 1.0f, 0.0f};
    IslandGraph routeGraph = makeGraph(
        {makeIsland(0), makeIsland(1), makeIsland(2)},
        {makeEdge(direct), makeEdge(firstHop), makeEdge(secondHop)});

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
    SUBCASE("Rejects invalid magic") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        std::string bytes = serialized.str();
        bytes[0] = 0;
        std::stringstream invalid(bytes, std::ios::in | std::ios::binary);
        CHECK(IslandGraphSerializer::read(invalid).status == SerializationStatus::InvalidMagic);
    }
    SUBCASE("Rejects unsupported version") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        std::string bytes = serialized.str();
        bytes[4] = 4;
        bytes[5] = 0;
        bytes[6] = 0;
        bytes[7] = 0;
        std::stringstream unsupported(bytes, std::ios::in | std::ios::binary);
        CHECK(IslandGraphSerializer::read(unsupported).status == SerializationStatus::UnsupportedVersion);
    }
    SUBCASE("Rejects previous directional-link cache version") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        std::string bytes = serialized.str();
        bytes[4] = 2;
        bytes[5] = 0;
        bytes[6] = 0;
        bytes[7] = 0;
        std::stringstream unsupported(bytes, std::ios::in | std::ios::binary);
        CHECK(IslandGraphSerializer::read(unsupported).status == SerializationStatus::UnsupportedVersion);
    }
    SUBCASE("Read limits are configurable") {
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, routeGraph) == SerializationStatus::Success);
        serialized.seekg(0);
        DeserializationLimits limits;
        limits.maxIslandCount = 2;
        CHECK(IslandGraphSerializer::read(serialized, limits).status == SerializationStatus::MalformedData);
    }
    SUBCASE("Rejects polygons owned by multiple islands") {
        IslandGraph invalidGraph({makeIsland(0, {42}), makeIsland(1, {42})});
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(IslandGraphSerializer::write(serialized, invalidGraph) == SerializationStatus::Success);
        serialized.seekg(0);
        CHECK(IslandGraphSerializer::read(serialized).status == SerializationStatus::MalformedData);
    }
    SUBCASE("Rejects non-finite island coordinates") {
        Island invalidIsland = makeIsland(0);
        invalidIsland.center.x = (std::numeric_limits<float>::quiet_NaN)();
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(
            IslandGraphSerializer::write(serialized, IslandGraph({invalidIsland})) ==
            SerializationStatus::Success);
        serialized.seekg(0);
        CHECK(IslandGraphSerializer::read(serialized).status == SerializationStatus::MalformedData);
    }
    SUBCASE("Rejects non-finite link coordinates") {
        Link invalidLink{0, 1};
        invalidLink.end.z = (std::numeric_limits<float>::infinity)();
        std::stringstream serialized(std::ios::in | std::ios::out | std::ios::binary);
        REQUIRE(
            IslandGraphSerializer::write(
                serialized,
                makeGraph({makeIsland(0), makeIsland(1)}, {makeEdge(invalidLink)})) ==
            SerializationStatus::Success);
        serialized.seekg(0);
        CHECK(IslandGraphSerializer::read(serialized).status == SerializationStatus::MalformedData);
    }
}
